#include "penal_traj_opt.h"

namespace planner
{
    void PenalTrajOpt::init(std::unordered_map<std::string, double> params)
    {
        grid_map = std::make_shared<GridMap>();
        grid_map->init(params);
        rho_T = 100.0;
        mid_vel = 1.0;
        g_epsilon = 1.0e-6;
        min_step = 1.0e-32;
        max_iter = 100000;
        delta = 1.0e-5;
        mem_size = 256;
        past = 3;
        int_K = 32;
        max_v = 1.0;
        rho_v = 10000.0;
        rho_collision = 100000.0;

        return;
    }

    void PenalTrajOpt::setMap(RowMatrixXi map)
    {
        grid_map->setMap(map);
        return;
    }

    bool PenalTrajOpt::plan(const Eigen::Vector3d& start, 
                            const Eigen::Vector3d& end, 
                            double safe_threshold_)
    {
        safe_threshold = safe_threshold_;
        
        auto astar_res = grid_map->astarPlan(start.head(2), end.head(2), safe_threshold);
        std::vector<Eigen::Vector2d> init_path = astar_res.first;
        if (init_path.empty())
            return false;
        
        // init solution
        Eigen::Matrix<double, 2, 2> init_pos, end_pos;
        init_pos.setZero();
        end_pos.setZero();
        init_pos.col(0) = init_path[0];
        end_pos.col(0) = init_path.back();
        init_pos.col(1)[0] = 0.1 * cos(start(2));
        init_pos.col(1)[1] = 0.1 * sin(start(2));
        end_pos.col(1)[0] = 0.1 * cos(end(2));
        end_pos.col(1)[1] = 0.1 * sin(end(2));
        Eigen::MatrixXd inner_pos;
        double total_time;
        Eigen::VectorXd times;

        double total_len = 0.0;
        double piece_len = 2.0;
        std::vector<Eigen::Vector2d> inner_pos_node;
        std::vector<double> times_node;
        for (size_t k=0; k<init_path.size()-1; k++)
        {
            double temp_seg = (init_path[k+1] - init_path[k]).head(2).norm();
            total_len += temp_seg;
        }
        int piece_num = (int) (total_len / piece_len);
        piece_len = total_len / piece_num;
        for (int i=0; i<piece_num; i++)
        {
            times_node.push_back(piece_len / mid_vel);
        }

        int cnt = 0;
        bool ok = false;
        double temp_len_pos = 0.0;
        for (size_t k=0; k<init_path.size()-1; k++)
        {
            double temp_seg = (init_path[k+1] - init_path[k]).norm();
            temp_len_pos += temp_seg;
            while (temp_len_pos > piece_len)
            {
                Eigen::Vector2d temp_node = init_path[k] + (1.0 - (temp_len_pos-piece_len) / temp_seg) * (init_path[k+1] - init_path[k]);
                inner_pos_node.push_back(temp_node);
                temp_len_pos -= piece_len;
                cnt++;
                if (cnt==piece_num-1)
                {
                    ok = true;
                    break;
                }
            }
            if (ok)
                break;
        }
        
        total_time = total_len / mid_vel;
        inner_pos.resize(2, inner_pos_node.size());
        times.resize(times_node.size());
        
        for (size_t i=0; i<inner_pos_node.size(); i++)
        {
            inner_pos.col(i) = inner_pos_node[i];
        }
        for (size_t i=0; i<times_node.size(); i++)
        {
            times(i) = times_node[i];
        }

        optimizeSE2Traj(init_pos, inner_pos, end_pos, total_time);
        
        // visualization
        SE2Trajectory back_end_traj = getTraj();
        std::vector<double> max_values = getMaxValues(back_end_traj);
        std::cout << "max v rate: "<< max_values[0] << std::endl;
        std::cout << "min sdf     : "<< max_values[1] << std::endl;
        return true;
    }

    static double lbfgsCallbackUni(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);
    static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                         const double fx, const double step, int k, int ls);
    int PenalTrajOpt::optimizeSE2Traj(const Eigen::MatrixXd &initPos, \
                                        const Eigen::MatrixXd &innerPtsPos , \
                                        const Eigen::MatrixXd &endPos, \
                                        const double & totalTime            )
    {
        in_opt = true;

        piece_pos = innerPtsPos.cols() + 1;
        minco_se2.reset(piece_pos);
        init_pos = initPos;
        end_pos = endPos;

        int variable_num = 2*(piece_pos-1) + 1;

        // init solution
        Eigen::VectorXd x;
        x.resize(variable_num);

        dim_T = 1;
        double& tau = x(0);
        Eigen::Map<Eigen::MatrixXd> Ppos(x.data() + dim_T, 2, piece_pos -1);

        tau = logC2(totalTime);
        Ppos = innerPtsPos;

        Eigen::VectorXd Tpos;
        Tpos.resize(piece_pos);
        calTfromTauUni(tau, Tpos);
        minco_se2.generate(init_pos, end_pos, Ppos, Tpos);
        std::vector<double> max_values = getMaxValues(getTraj());
        std::cout << "At beginning of Optimization: "<< std::endl;
        std::cout << "max v rate: "<< max_values[0] << std::endl;
        std::cout << "min sdf     : "<< max_values[1] << std::endl;

        // lbfgs params
        lbfgs::lbfgs_parameter_t lbfgs_params;
        lbfgs_params.mem_size = mem_size;
        lbfgs_params.past = past;
        lbfgs_params.g_epsilon = g_epsilon;
        lbfgs_params.min_step = min_step;
        lbfgs_params.delta = delta;
        lbfgs_params.max_iterations = max_iter;
        double lbfgs_cost;

        // begin L-BFGS Method
        int result = lbfgs::lbfgs_optimize(x, lbfgs_cost, &lbfgsCallbackUni, nullptr, 
                                            &earlyExit, this, lbfgs_params);

        if (result == lbfgs::LBFGS_CONVERGENCE ||
            result == lbfgs::LBFGS_CANCELED ||
            result == lbfgs::LBFGS_STOP || 
            result == lbfgs::LBFGSERR_MAXIMUMITERATION)
        {
            PRINTF_WHITE("[L-BFGS] optimization success! return " << result <<"\n");
        }
        else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
        {
            PRINT_YELLOW("[L-BFGS] The line-search routine reaches the maximum number of evaluations.");
        }
        else
        {
            PRINT_RED("[L-BFGS] Solver error. Return = " << result << lbfgs::lbfgs_strerror(result));
        }
    
        in_opt = false;
        
        return result;
    }

    static double lbfgsCallbackUni(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
    {
        PenalTrajOpt& obj = *(PenalTrajOpt*)ptrObj;
        double cost = 0.0;

        // get x
        const double& tau = x(0);
        double& grad_tau = grad(0);
        Eigen::Map<const Eigen::MatrixXd> Ppos(x.data() + obj.dim_T, 2, obj.piece_pos - 1);
        Eigen::Map<Eigen::MatrixXd> gradPpos(grad.data() + obj.dim_T, 2, obj.piece_pos - 1);

        // get T from τ, generate MINCO trajectory
        Eigen::VectorXd Tpos;
        Tpos.resize(obj.piece_pos);
        obj.calTfromTauUni(tau, Tpos);
        obj.minco_se2.generate(obj.init_pos, obj.end_pos, Ppos, Tpos);

        // get acc cost with grad (C,T)
        double acc_cost = 0.0;
        Eigen::MatrixXd gdCpos_acc;
        Eigen::VectorXd gdTpos_acc;
        obj.minco_se2.calAccGradCT(gdCpos_acc, gdTpos_acc);
        acc_cost = obj.minco_se2.getTrajAccCost();
        acc_cost = 0;
        gdCpos_acc.setZero();
        gdTpos_acc.setZero();

        // get constrain cost with grad (C,T)
        double constrain_cost = 0.0;
        Eigen::MatrixXd gdCpos_constrain;
        Eigen::VectorXd gdTpos_constrain;
        obj.calConstrainCostGrad(constrain_cost, gdCpos_constrain, gdTpos_constrain);

        // get grad (q, T) from (C, T)
        Eigen::MatrixXd gdCpos = gdCpos_acc + gdCpos_constrain;
        Eigen::VectorXd gdTpos = gdTpos_acc + gdTpos_constrain;
        Eigen::MatrixXd gradPpos_temp;
        Eigen::MatrixXd gradPtail_temp;
        obj.minco_se2.calGradCTtoQT(gdCpos, gdTpos, gradPpos_temp, gradPtail_temp);
        gradPpos = gradPpos_temp;

        // get tau cost with grad
        double tau_cost = obj.rho_T * obj.expC2(tau);
        double grad_Tsum = obj.rho_T+ \
                           gdTpos.sum() / obj.piece_pos;
        grad_tau = grad_Tsum * obj.getTtoTauGrad(tau);

        cost = acc_cost + constrain_cost + tau_cost;

        return cost;
    }

    void PenalTrajOpt::calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCpos, Eigen::VectorXd &gdTpos)
    {
        cost = 0.0;
        gdCpos.resize(4*piece_pos, 2);
        gdCpos.setZero();
        gdTpos.resize(piece_pos);
        gdTpos.setZero();

        Eigen::Vector2d pos, vel, acc;
        double grad_time = 0.0;
        Eigen::Vector2d grad_p = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_v = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_sdf = Eigen::Vector2d::Zero();
        Eigen::Matrix<double, 4, 1> beta0, beta1, beta2;
        double s1, s2, s3;
        double step, alpha, omg;

        for (int i=0; i<piece_pos; i++)
        {
            const Eigen::Matrix<double, 4, 2> &c = minco_se2.se2_opt.getCoeffs().block<4, 2>(i * 4, 0);
            step = minco_se2.se2_opt.T1(i) / int_K;
            s1 = 0.0;

            for (int j=0; j<=int_K; j++)
            {
                alpha = 1.0 / int_K * j;

                // set zero
                grad_p.setZero();
                grad_v.setZero();
                grad_sdf.setZero();
                grad_time = 0.0;

                // analyse xy
                s2 = s1 * s1;
                s3 = s2 * s1;
                beta0 << 1.0, s1, s2, s3;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1;
                pos = c.transpose() * beta0;
                vel = c.transpose() * beta1;
                acc = c.transpose() * beta2;

                omg = (j == 0 || j == int_K) ? 0.5 : 1.0;

                double vxy_snorm = vel.head(2).squaredNorm();
                double sdf_value;
                grid_map->getDisWithGradI(pos, sdf_value, grad_sdf);
                
                // vxy
                double vViola = vxy_snorm - max_v * max_v;
                if (vViola > 0) 
                {
                    grad_v.head(2) += rho_v * 6 * vViola * vViola * vel;
                    double cost_v = rho_v * vViola * vViola * vViola;
                    cost += cost_v * omg * step;
                    grad_time += omg * (cost_v / int_K + step * alpha * grad_v.dot(acc));
                }

                // collision
                double cViola = safe_threshold - sdf_value;
                if (cViola > 0)
                {
                    double fcViola;
                    double gcViola;
                    smoothL1Penalty(cViola, fcViola, gcViola);
                    Eigen::Vector2d grad_pc = -rho_collision * gcViola * grad_sdf;
                    double cost_c = rho_collision * fcViola;
                    cost += cost_c * omg * step;
                    grad_time += omg * (cost_c / int_K + step * alpha * grad_pc.dot(vel));
                    grad_p += grad_pc;
                }

                // add all grad into C,T
                // note that p = C*β(j/K*T)
                // ∂p/∂C, ∂v/∂C
                gdCpos.block<4, 2>(i * 4, 0) += (beta0 * grad_p.transpose() + \
                                                beta1 * grad_v.transpose()) * omg * step;
                // ∂p/∂Txy, ∂v/∂Txy, ∂a/∂Txy
                gdTpos(i) += grad_time;
                
                s1 += step;
            }
        }

        return;
    }

    static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                         const double fx, const double step, int k, int ls)
    {
        PenalTrajOpt& obj = *(PenalTrajOpt*)ptrObj;
        return k > 1e3;
    }

}