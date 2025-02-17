#pragma once

#include <thread>
#include <numeric>
#include <iostream>
#include <fstream>

#include "utils/se2traj.hpp"
#include "utils/lbfgs.hpp"
#include "grid_map.h"

using namespace std;

namespace planner
{
    class PenalTrajOpt
    {
        public:
            // params

            /// problem
            bool use_uniform;
            double rho_T;
            double mid_vel;
            double max_v;
            double safe_threshold;
            double rho_v;
            double rho_collision;

            /// L-BFGS
            double g_epsilon;
            double min_step;
            double max_iter;
            double delta;
            int    mem_size;
            int    past;
            int    int_K;

            /// debug and test
            bool in_test;
            bool in_debug;
            bool in_opt = false;

            // data
            int             piece_pos;
            int             dim_T;
            Eigen::MatrixXd init_pos;
            Eigen::MatrixXd end_pos;
            SE2Opt          minco_se2;
            GridMap::Ptr    grid_map;
            
        public:
            void init(std::unordered_map<std::string, double> params);
            void setMap(RowMatrixXi map);
            bool plan(const Eigen::Vector3d& start, 
                    const Eigen::Vector3d& end, 
                    double safe_threshold = 0.0);
            int optimizeSE2Traj(const Eigen::MatrixXd &initPos, \
                                const Eigen::MatrixXd &innerPtsPos , \
                                const Eigen::MatrixXd &endPos, \
                                const double & totalTime            );

            void calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCpos, Eigen::VectorXd &gdTpos);

            inline SE2Trajectory getTraj();
            inline vector<double> getMaxValues(const SE2Trajectory& traj);

            // process with T and τ
            inline double expC2(const double& tau);
            inline double logC2(const double& T);
            inline double getTtoTauGrad(const double& tau);
            inline void calTfromTauUni(const double& tau, Eigen::VectorXd& T);
            inline void calTfromTau(const Eigen::VectorXd& tau, Eigen::VectorXd& T);

            // L1 penalty
            inline void smoothL1Penalty(const double& x, double& f, double &df);
    };

    inline SE2Trajectory PenalTrajOpt::getTraj()
    {
        return minco_se2.getTraj();
    }

    inline vector<double> PenalTrajOpt::getMaxValues(const SE2Trajectory& traj)
    {
        double max_v_ = 0.0;
        double min_sdf_ = 1e+10;

        double dt = 0.01;
        for(double t = 0.0; t < traj.getTotalDuration(); t += dt)
        {
            Eigen::Vector3d p = traj.getPos(t);
            Eigen::Vector2d v = traj.getVel(t);
            
            double sdf;
            grid_map->getDistance(p.head(2), sdf);

            if(fabs(max_v_) < v.head(2).norm())
            {
                max_v_ = v.head(2).norm();
            }

            if (min_sdf_ > sdf)
            {
                min_sdf_ = sdf;
            }
        }

        return vector<double>{max_v_, min_sdf_};
    }

    // T = e^τ
    inline double PenalTrajOpt::expC2(const double& tau)
    {
        return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
    }

    // τ = ln(T)
    inline double PenalTrajOpt::logC2(const double& T)
    {
        return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
    }

    // get dT/dτ
    inline double PenalTrajOpt::getTtoTauGrad(const double& tau)
    {
        if (tau > 0)
            return tau + 1.0;
        else 
        {
            double denSqrt = (0.5 * tau - 1.0) * tau + 1.0;
            return (1.0 - tau) / (denSqrt * denSqrt);
        } 
    }

    // know τ
    // then get T (uniform)
    inline void PenalTrajOpt::calTfromTauUni(const double& tau, Eigen::VectorXd& T)
    {
        T.setConstant(expC2(tau) / T.size());
        return;
    }

    // know τ
    // then get T
    inline void PenalTrajOpt::calTfromTau(const Eigen::VectorXd& tau, Eigen::VectorXd& T)
    {
        T.resize(tau.size());
        for (int i=0; i<tau.size(); i++)
        {
            T(i) = expC2(tau(i));
        }
        return;
    }

    // reLu
    inline void PenalTrajOpt::smoothL1Penalty(const double& x, double& f, double &df)
    {
        const double miu = 1.0e-4;
        
        if (x > miu)
        {
            df = 1.0; 
            f =  x - 0.5 * miu;
        }
        else
        {
            double xdmu = x / miu;
            double sqrxdmu = xdmu * xdmu;
            double mumxd2 = miu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / miu);
        }

        return;
    }
}