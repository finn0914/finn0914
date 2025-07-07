#include "controllers/controllers.hpp"

using namespace Controllers;
PID::PID(double kp, double ki, double kd):kp_{kp},ki_{ki},kd_{kd}{
    std::cout<<"***************pid coefficients are setted!*************88"<<std::endl;

}
PID::~PID(){
}
double PID::compute(double setpoint,double measured_value,double dt){
    try{
        double error = setpoint - measured_value;
        // Proportional term
        double proportional = kp_ * error;
        // Integral term
        integral_ += error * dt;
        double integral = ki_ * integral_;
        // Derivative term
        double derivative = kd_ * (error - prev_error_) / dt;
        // Save error for next computation
        prev_error_ = error;
        // Return total control output
        return proportional + integral + derivative;
    }
    catch(std::exception &ex){
        std::cout<<ex.what()<<std::endl;
    }

}
void PID::set_coefficients(double kp, double ki, double kd){
    kp_=kp;
    ki_=ki;
    kd_=kd;
    // std::cout<<"***************pid coefficients are set!*************88"<<std::endl;

}
void PID::print_coefficients(){
        std::cout<<"******PID coefficients are : P["<<PID::kp_<<"] I["<<PID::ki_<<"] D["<<PID::kd_<<"] ************"<<std::endl;

}