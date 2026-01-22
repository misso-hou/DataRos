#include "algorithm/observer.h"

namespace ALG{

using namespace rls_filter;
using namespace Eigen;

template <typename T, int N>
using VectorXt = Matrix<T, N, 1>;

//********************************************纵向力观测器********************************************/
/**
 * step01->estimate brake gain(RLS)[RLS又需要用到纵向力去迭代估计]
 * step02->calculate brake torque(brake pressure*brake gain)
 * step03->observer estimate wheel force Fx
 * step04->estimate acceleration compensation factor(RLS) [补偿ebs控制器造成的加速度误差],适当降低任务难度,可以先辨识刹车性能下降
 */

/**
 * @brief:initilize luenber observer(wheel longitudinal force observer)
 */
void BrakeTorqueObserver::initBrakeObserver(){
    //TODO:set vehicle params
    // model_params_.drag_coeff = raw_param_->drag_coefficient();
    // model_params_.front_area = raw_param_->vehicle_frontal_area();
    // model_params_.rolling_friction_coeff = raw_param_->rolling_friction_coefficient();
    // TEST
    model_params_.ts = 0.05;
    model_params_.wheel_inertia = 100.0;
    model_params_.wheel_radius = 0.6;
    model_params_.brake_gain = 0.6; 
    model_params_.drag_coeff = 0.68;       //aero drag 
    model_params_.front_area = 10.0;
    model_params_.rolling_friction_coeff = 0.007;
    model_params_.mass = 29000.0;
    //state space matrices
    mat_a_ = Eigen::MatrixXd::Zero(2,2);
    mat_b_ = Eigen::MatrixXd::Zero(2,1);
    mat_c_ = Eigen::MatrixXd::Zero(2,2);
    mat_a_(0,0) = 1.0;
    mat_a_(1,0) = -model_params_.wheel_radius * model_params_.ts/model_params_.wheel_inertia;
    mat_b_(1,0) = model_params_.ts/model_params_.wheel_inertia;
    mat_c_(1,1) = 1.0;
    mat_L_ = Eigen::MatrixXd::Zero(2,1);
    mat_L_(0,0) = -500;
    mat_L_(1,0) = 1;
    x_hat_ = Eigen::VectorXd::Zero(2);
    y_mes_ = Eigen::VectorXd::Zero(2);
    brake_gain_RLS_ = std::make_unique<RLSFilter<double,1>>(1.0,1.0);
    Eigen::Matrix<double, 1, 1> w0;
    brake_gain_RLS_->setEstimatedCoefficients(w0);
}


/**
 * @brief:propogate the state space model
 * wheel dynamic-> I*omege_w_dot - R*Fx + T_brake = 0
 * state->(Fx,omega_wheel,)
 */
void BrakeTorqueObserver::stateUpdate(const double& pressure,
                                      const double& acceleration,
                                      const double& wheel_speed){
    //step0->update measurement
    y_mes_(1,0) = wheel_speed;                                  
    //step1->update state space matrices(time invariant,ignore)
    mat_a_(0,0) = 1.0;
    mat_a_(1,0) = -model_params_.wheel_radius * model_params_.ts/model_params_.wheel_inertia;
    mat_b_(1,0) = model_params_.ts/model_params_.wheel_inertia;
    mat_c_(1,1) = 1.0;
    //step2->first order approximation of ODE                                    
    double brake_torque = brake_gain_ * pressure;
    auto y_error = y_mes_ - mat_c_ * x_hat_;
    x_hat_ += (mat_a_ * x_hat_ + mat_b_ * brake_torque) + mat_L_ * y_error;
    tyre_road_force_ = x_hat_(0,0);
}

/**
 * @brief:calculate coasting force based on formula(aero drag,rolling resistence,gravity component)
 */
double BrakeTorqueObserver::computeCoastingForce(const double& v) {
    double aero_drag = ComputeAeroDrag(v);           
    double roll_resistance = ComputeRollingResis();   // REVIEW:
    // double F_grav = ComputeGravityLoad(vehicle_mass, pitch);
    // double F_total = - F_aero - F_roll - F_grav;
    double total_force = -aero_drag - roll_resistance;      //ignore insignificant external force
    // std::cout << "debug->aero drag:" << aero_drag << "; rolling resistence:" << roll_resistance << std::endl;
    return total_force;
}

double BrakeTorqueObserver::ComputeAeroDrag(double v) {
    return 0.5 * model_params_.drag_coeff * model_params_.front_area * AIR_DENSITY * v * v;
}

double BrakeTorqueObserver::ComputeRollingResis() {
    return model_params_.rolling_friction_coeff * model_params_.mass * GRAVITY_ACCELERATION;
}

/**
 * @brief:estimate brake gain(in brake state)
 * note:是否需要考虑缓速器的影响???
 */
double BrakeTorqueObserver::estimateBrakeGain(const double& v,
                                            const double& acc_mes, 
                                            const double& pressure){
    //calculate coasting force
    double coasting_force = computeCoastingForce(v);
    //calculate tyre-road contact force(longitudinal force - coasting force) 
    double virtual_brake_torque = (acc_mes * model_params_.mass + coasting_force) * model_params_.wheel_radius;
    //calculate measured/real brake gain
    if(fabs(pressure) > 50){
        // virtual_brake_gain = virtual_brake_torque / pressure;   //(brake_torque-wheel_inertia_torque)/pressure
        // std::cout << "debug->virtual_brake_torque:" << virtual_brake_torque << "; pressure:" << pressure<<  "; virtual_brake_gain:" << virtual_brake_gain << std::endl;
        Eigen::Matrix<double,1,1> new_x;
        new_x << pressure;
        brake_gain_RLS_->update(new_x,virtual_brake_torque);
        // auto updated_brake_gain = brake_gain_RLS_->estimatedCoefficients();                                                 
        // brake_gain_ = updated_brake_gain[0];   
        std::cout << "debug-> RLS brake gain:" << brake_gain_ << std::endl;
    }                 
    auto updated_brake_gain = brake_gain_RLS_->estimatedCoefficients();                                                 
    brake_gain_ = updated_brake_gain[0];   
    return brake_gain_;                                        
}

//********************************************横向观测器********************************************/
/**
 * @brief:initilize luenber observer(wheel longitudinal force observer)
 */
// void LateralObserver::initBrakeObserver(){
//     //set threshold
//     min_v_thd_ = 0.0;
//     //TODO:set vehicle params
//     // model_params_.drag_coeff = raw_param_->drag_coefficient();
//     // model_params_.front_area = raw_param_->vehicle_frontal_area();
//     // model_params_.rolling_friction_coeff = raw_param_->rolling_friction_coefficient();
//     // TEST
//     model_params_.ts = 0.0;
//     model_params_.cf = 0.0;
//     model_params_.cr = 0.0;
//     model_params_.lf = 0.0; 
//     model_params_.lr = 0.0;
//     model_params_.Iz = 0.0;
//     model_params_.mass = 0.0;
//     model_params_.v = 0.0;
//     model_params_.steering_tau = 0.0;
//     model_params_.swa_to_delta_ratio = 25.0;
//     //state space matrices
//     mat_a_ = Eigen::MatrixXd::Zero(4,4);
//     mat_b_ = Eigen::MatrixXd::Zero(4,1);
//     mat_c_ = Eigen::MatrixXd::Zero(4,4);
//     // state propagation matrix A
//     mat_a_(0,0) = -2*(model_params_.cf + model_params_.cr) / (model_params_.mass * model_params_.v);
//     mat_a_(0,1) = -model_params_.v + 2*(model_params_.lr * model_params_.cr - model_params_.lf * model_params_.cf) / (model_params_.mass * model_params_.v);
//     mat_a_(0,2) = 2*model_params_.cf / model_params_.mass;
//     mat_a_(1,0) = 2*(model_params_.lr * model_params_.cr - model_params_.lf * model_params_.cf) / (model_params_.iz * model_params_.v);
//     mat_a_(1,1) = -2*(model_params_.lf * model_params_.lf * model_params_.cf + model_params_.lr * model_params_.lr * model_params_.cr) / (model_params_.iz * model_params_.v);
//     mat_a_(1,2) = 2*model_params_.lf * model_params_.cf / model_params_.iz;
//     mat_a_(2,2) = -1 / model_params_.steering_tau;
//     mat_a_(2,3) = 1 / model_params_.steering_tau;   // REVIEW:车轮与方向盘转角转换
//     // input matrix B
//     mat_b_(2,0) = 1 / model_params_.steering_tau;
//     // matrix C
//     mat_c_(1,1) = 1;
//     // observer matrix L //TODO:
//     mat_L_ = Eigen::MatrixXd::Zero(4,1);
//     mat_L_(0,0) = 0.0;
//     mat_L_(1,0) = 0.0;
//     mat_L_(2,0) = 0.0;
//     mat_L_(3,0) = 0.0;
//     x_hat_ = Eigen::VectorXd::Zero(4,1);
//     y_mes_ = Eigen::VectorXd::Zero(4,1);
// }

// void LateralObserver::resetObserver(const double& steer_angle){
//     x_hat_(0, 0) = 0.0;
//     x_hat_(1, 0) = model_params_.yaw_rate();
//     x_hat_(2, 0) = steer_angle;
// }

// /**
//  * @brief:propogate the state space model
//  * wheel dynamic-> I*omege_w_dot - R*Fx + T_brake = 0
//  * state->(Fx,omega_wheel,)
//  */
// void LateralObserver::stateUpdate(const double& speed,
//                                   const double& yaw_rate,
//                                   const double& steering_wheel_angle){
//     //step0->set model params
//     model_params_.v = speed;
//     model_params_.yaw_rate = yaw_rate;
//     //TODO:update steering tau by interpolation???
//     //step01->update measurement(front wheel steering angle) //TODO:ebs compensation
//     auto delta_angle = steering_wheel_angle / model_params_.swa_to_delta_ratio;
//     if(model_params_.v < min_v_thd_){
//         resetObserver(delta_angle);
//     }                        
//     //step02->update state space matrices time variant variables
//     mat_a_(0,0) = -2*(model_params_.cf + model_params_.cr) / (model_params_.mass * model_params_.v);
//     mat_a_(0,1) = -model_params_.v + 2*(model_params_.lr * model_params_.cr - model_params_.lf * model_params_.cf) / (model_params_.mass * model_params_.v);
//     mat_a_(1,0) = 2*(model_params_.lr * model_params_.cr - model_params_.lf * model_params_.cf) / (model_params_.iz * model_params_.v);
//     mat_a_(1,1) = -2*(model_params_.lf * model_params_.lf * model_params_.cf + model_params_.lr * model_params_.lr * model_params_.cr) / (model_params_.iz * model_params_.v);
//     mat_b_(2,0) = 1 / model_params_.steering_tau;
//     //step3->first order approximation of ODE
//     auto y_err = yaw_rate - x_hat_(1);
//     x_hat_ += model_params_.ts * (mat_a_ * x_hat_ + mat_b_ * delta_angle) + mat_L_ * y_err;                                   
// }

// /**
//  * @brief:steering first order delay variable time 
//  */
// void LateralObserver::setSteerTau(const double& steer_tau){
//     model_params_.steer_tau = steer_tau;
// }

// double LateralObserver::getSteerAngle(){
//     return x_hat_(2);
// }

// double LateralObserver::getSteeringWheelOffsetAngle(){
//     auto offset_angle = x_hat(3) * model_params_.swa_to_delta_ratio;
// }


}

