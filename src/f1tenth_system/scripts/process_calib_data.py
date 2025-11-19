#!/usr/bin/env python3
"""
Post-processing script for current-acceleration calibration data

This script processes rosbag files or CSV data from the calibration run
to extract and analyze the relationship between motor current and acceleration.

Usage:
  python3 process_calib_data.py --input data.csv --output results.npz
"""

import numpy as np
import argparse
from scipy.optimize import curve_fit
from scipy import stats
import matplotlib.pyplot as plt
import json
from pathlib import Path


class CalibrationProcessor:
    """Process and analyze calibration data"""
    
    def __init__(self):
        self.stages = [
            {"name": "STAGE_1", "current": 5.0, "t_start": 0, "t_end": 30},
            {"name": "STAGE_2", "current": 10.0, "t_start": 30, "t_end": 60},
            {"name": "STAGE_3", "current": 15.0, "t_start": 60, "t_end": 90},
            {"name": "STAGE_4", "current": 20.0, "t_start": 90, "t_end": 120},
        ]
    
    def load_data(self, filepath):
        """
        Load calibration data from CSV file.
        
        Expected CSV columns:
          timestamp, current, velocity, acceleration, steering_angle, cross_track_error
        
        Args:
            filepath: Path to CSV file
            
        Returns:
            dict: Loaded data with keys 'timestamp', 'current', etc.
        """
        data = np.genfromtxt(filepath, delimiter=',', names=True)
        return data
    
    @staticmethod
    def linear_model(I, k, b):
        """Linear calibration model: a = k*I + b"""
        return k * I + b
    
    @staticmethod
    def quadratic_model(I, k1, k2, b):
        """Quadratic model: a = k1*I + k2*I^2 + b"""
        return k1 * I + k2 * I**2 + b
    
    def fit_stage_data(self, currents, accelerations, stage_name, model='linear'):
        """
        Fit acceleration model for a single calibration stage.
        
        Args:
            currents: Array of current values (A)
            accelerations: Array of acceleration values (m/s^2)
            stage_name: Name of stage for logging
            model: 'linear' or 'quadratic'
            
        Returns:
            dict: Fitting results including parameters and statistics
        """
        # Remove outliers using IQR method
        Q1 = np.percentile(accelerations, 25)
        Q3 = np.percentile(accelerations, 75)
        IQR = Q3 - Q1
        mask = (accelerations > Q1 - 1.5*IQR) & (accelerations < Q3 + 1.5*IQR)
        
        currents_clean = currents[mask]
        accelerations_clean = accelerations[mask]
        
        if len(currents_clean) < 5:
            print(f"  Warning: {stage_name} has too few valid samples ({len(currents_clean)})")
            return None
        
        # Perform fitting
        try:
            if model == 'linear':
                popt, pcov = curve_fit(self.linear_model, currents_clean, 
                                      accelerations_clean)
                params = {'k': popt[0], 'b': popt[1]}
                perr = np.sqrt(np.diag(pcov))
                param_errs = {'k_err': perr[0], 'b_err': perr[1]}
                
            elif model == 'quadratic':
                popt, pcov = curve_fit(self.quadratic_model, currents_clean,
                                      accelerations_clean)
                params = {'k1': popt[0], 'k2': popt[1], 'b': popt[2]}
                perr = np.sqrt(np.diag(pcov))
                param_errs = {'k1_err': perr[0], 'k2_err': perr[1], 'b_err': perr[2]}
            
            # Compute fit quality metrics
            if model == 'linear':
                y_pred = self.linear_model(currents_clean, popt[0], popt[1])
            else:
                y_pred = self.quadratic_model(currents_clean, popt[0], popt[1], popt[2])
            
            residuals = accelerations_clean - y_pred
            ss_res = np.sum(residuals**2)
            ss_tot = np.sum((accelerations_clean - np.mean(accelerations_clean))**2)
            r_squared = 1 - (ss_res / ss_tot)
            rmse = np.sqrt(np.mean(residuals**2))
            
            result = {
                'stage': stage_name,
                'model': model,
                'parameters': params,
                'param_errors': param_errs,
                'r_squared': r_squared,
                'rmse': rmse,
                'n_samples': len(currents_clean),
                'mean_acceleration': np.mean(accelerations_clean),
                'std_acceleration': np.std(accelerations_clean),
            }
            
            return result
            
        except Exception as e:
            print(f"  Error fitting {stage_name}: {e}")
            return None
    
    def process_data(self, data, model='linear'):
        """
        Process complete calibration dataset.
        
        Args:
            data: Loaded data dictionary
            model: 'linear' or 'quadratic'
            
        Returns:
            dict: Results for all stages
        """
        print(f"\nProcessing calibration data ({model} model)...")
        print("=" * 70)
        
        results = {}
        
        for stage in self.stages:
            print(f"\n{stage['name']} ({stage['current']}A):")
            print("-" * 70)
            
            # Extract data for this stage
            time_mask = (data['timestamp'] >= stage['t_start']) & \
                       (data['timestamp'] < stage['t_end'])
            
            if np.sum(time_mask) == 0:
                print(f"  No data for this stage!")
                continue
            
            stage_current = np.full(np.sum(time_mask), stage['current'])
            stage_accel = data['acceleration'][time_mask]
            
            # Fit model
            result = self.fit_stage_data(stage_current, stage_accel, 
                                        stage['name'], model)
            
            if result is not None:
                results[stage['name']] = result
                
                # Print results
                if model == 'linear':
                    print(f"  Model: a = {result['parameters']['k']:.4f} * I + "
                          f"{result['parameters']['b']:.4f}")
                    print(f"  Error: k ± {result['param_errors']['k_err']:.4f}, "
                          f"b ± {result['param_errors']['b_err']:.4f}")
                else:
                    print(f"  Model: a = {result['parameters']['k1']:.4f} * I + "
                          f"{result['parameters']['k2']:.6f} * I^2 + "
                          f"{result['parameters']['b']:.4f}")
                
                print(f"  R² = {result['r_squared']:.4f}")
                print(f"  RMSE = {result['rmse']:.4f} m/s²")
                print(f"  Samples: {result['n_samples']}")
                print(f"  Mean accel: {result['mean_acceleration']:.4f} m/s²")
        
        return results
    
    @staticmethod
    def save_results(results, output_path):
        """Save results to JSON and NPZ files"""
        output_path = Path(output_path)
        
        # Save as JSON for human readability
        json_results = {}
        for stage_name, result in results.items():
            json_results[stage_name] = {
                'model': result['model'],
                'parameters': result['parameters'],
                'param_errors': result['param_errors'],
                'r_squared': float(result['r_squared']),
                'rmse': float(result['rmse']),
                'n_samples': int(result['n_samples']),
                'mean_acceleration': float(result['mean_acceleration']),
                'std_acceleration': float(result['std_acceleration']),
            }
        
        json_path = output_path.with_suffix('.json')
        with open(json_path, 'w') as f:
            json.dump(json_results, f, indent=2)
        print(f"\nResults saved to: {json_path}")
        
        # Save as NPZ for programmatic use
        npz_data = {}
        for stage_name, result in results.items():
            for key, value in result['parameters'].items():
                npz_data[f"{stage_name}_{key}"] = value
        
        npz_path = output_path.with_suffix('.npz')
        np.savez(npz_path, **npz_data)
        print(f"Results saved to: {npz_path}")
    
    @staticmethod
    def plot_results(results, output_path=None):
        """
        Generate visualization plots of calibration results.
        
        Args:
            results: Processing results dictionary
            output_path: Optional path to save plot
        """
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Current-Acceleration Calibration Results', fontsize=14)
        
        for idx, (stage_name, result) in enumerate(results.items()):
            ax = axes[idx // 2, idx % 2]
            
            # Generate synthetic data for visualization
            I_range = np.linspace(0, 25, 100)
            if result['model'] == 'linear':
                a_range = CalibrationProcessor.linear_model(
                    I_range, 
                    result['parameters']['k'],
                    result['parameters']['b']
                )
            else:
                a_range = CalibrationProcessor.quadratic_model(
                    I_range,
                    result['parameters']['k1'],
                    result['parameters']['k2'],
                    result['parameters']['b']
                )
            
            # Plot
            ax.plot(I_range, a_range, 'b-', linewidth=2, label='Fitted model')
            ax.axhline(y=result['mean_acceleration'], color='g', 
                      linestyle='--', alpha=0.7, label='Mean acceleration')
            ax.fill_between(I_range, 
                            result['mean_acceleration'] - result['std_acceleration'],
                            result['mean_acceleration'] + result['std_acceleration'],
                            alpha=0.2, color='green')
            
            ax.set_xlabel('Current (A)', fontsize=11)
            ax.set_ylabel('Acceleration (m/s²)', fontsize=11)
            ax.set_title(f"{stage_name} (R² = {result['r_squared']:.3f})", fontsize=12)
            ax.grid(True, alpha=0.3)
            ax.legend()
        
        plt.tight_layout()
        
        if output_path:
            plt.savefig(output_path, dpi=150)
            print(f"Plot saved to: {output_path}")
        else:
            plt.show()
    
    @staticmethod
    def generate_calibration_curve(results):
        """
        Generate unified calibration curve across all stages.
        
        Returns:
            dict: Coefficients for piecewise linear or global quadratic model
        """
        print("\nGenerating unified calibration curve...")
        print("=" * 70)
        
        all_I = []
        all_params = []
        
        for stage_name in sorted(results.keys()):
            result = results[stage_name]
            if result['model'] == 'linear':
                I = result['parameters']['k']
                b = result['parameters']['b']
                all_I.append(I)
                all_params.append(b)
        
        if all_I:
            mean_k = np.mean(all_I)
            mean_b = np.mean(all_params)
            print(f"\nUnified Linear Model:")
            print(f"  a = {mean_k:.4f} * I + {mean_b:.4f}")
            print(f"  Valid current range: 5-20 A")
            print(f"  Expected acceleration range: "
                  f"{mean_k*5 + mean_b:.2f} to {mean_k*20 + mean_b:.2f} m/s²")
            
            return {
                'model_type': 'linear',
                'k': mean_k,
                'b': mean_b,
            }
        
        return None


def main():
    parser = argparse.ArgumentParser(
        description='Process F1TENTH current-acceleration calibration data'
    )
    parser.add_argument('--input', type=str, required=True,
                       help='Input CSV file path')
    parser.add_argument('--output', type=str, default='calib_results',
                       help='Output file prefix (without extension)')
    parser.add_argument('--model', type=str, default='linear',
                       choices=['linear', 'quadratic'],
                       help='Fitting model type')
    parser.add_argument('--plot', action='store_true',
                       help='Generate visualization plots')
    
    args = parser.parse_args()
    
    # Process data
    processor = CalibrationProcessor()
    
    try:
        data = processor.load_data(args.input)
        results = processor.process_data(data, model=args.model)
        
        if results:
            processor.save_results(results, args.output)
            
            if args.plot:
                plot_path = Path(args.output).with_suffix('.png')
                processor.plot_results(results, str(plot_path))
            
            # Generate unified calibration curve
            unified_curve = processor.generate_calibration_curve(results)
            
    except Exception as e:
        print(f"Error processing data: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
