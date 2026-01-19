#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "RLS_filter.h"

namespace drive {
namespace control {

const double GRAVITY_ACCELERATION = 9.81;
const double AIR_DENSITY = 1.225;  // kg/m^3

struct BrakeModelParams{
    double ts = 0.0;
    double wheel_inertia = 0.0;
    double wheel_radius = 0.0;
    double brake_gain; 
    double drag_coeff;
    double front_area;
    double rolling_friction_coeff;
    double mass;
};

struct LateralModelParams{
    double ts = 0.0;
    double cf = 0.0;
    double cr = 0.0;
    double lf; 
    double lr;
    double Iz;
    double mass;
    double v;
    double yaw_rate;
    double steering_tau;
    double swa_to_delta_ratio;
};

class BrakeTorqueObserver{
    public:
        BrakeTorqueObserver(){
            initBrakeObserver();
        }
        ~BrakeTorqueObserver(){}
    
    public:
        double estimateBrakeGain(const double& v,
            const double& acc_mes, 
            const double& pressure);

    private:
        void initBrakeObserver();
        void stateUpdate(const double& pressure,
                        const double& acceleration,
                        const double& wheel_speed);
        double computeCoastingForce(const double& v);
        double ComputeAeroDrag(double v);
        double ComputeRollingResis();

    private: //internal 
        double brake_gain_ = 0.0;  //ratio between brake pressure and brake force
        double tyre_road_force_ = 0.0;
        BrakeModelParams model_params_;
        std::unique_ptr<rls_filter::RLSFilter<double,1>> brake_gain_RLS_;


    private:  //state space matrices
        Eigen::MatrixXd mat_a_;
        Eigen::MatrixXd mat_b_;
        Eigen::MatrixXd mat_c_;
        Eigen::MatrixXd mat_L_;
        Eigen::VectorXd x_hat_;  //(brake force, wheel speed)
        Eigen::VectorXd y_mes_;      
};


/**
 * 横向观测器
 * link:https://www.wolai.com/oeWghNskTdDff5pzJ62fA5
 */
class LateralObserver {
    public:
        LaterialObserver(){
            initLateralObserver();
        }
        ~LateralObserver(){}

    public:
        void stateUpdate(const double& steering_wheel_angle);

    private: //internal 
        LateralModelParams model_params_;
        double min_v_thd_;

    private:  //state space matrices
        Eigen::MatrixXd mat_a_;
        Eigen::MatrixXd mat_b_;
        Eigen::MatrixXd mat_c_;
        Eigen::MatrixXd mat_L_;
        Eigen::VectorXd x_hat_;
        Eigen::VectorXd y_mes_;      
}

}
}