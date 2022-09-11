#include "generic_pid.h"
#include <stdint.h>

namespace controllib{

       generic_pid::generic_pid(double _kp, double _ki, double _kd, double _bias,double _T):kp(_kp),ki(_ki),kd(_kd),bias(_bias),T(_T){
            
        }
        void generic_pid::set_kp(double _kp){
            kp=_kp;
        }
        void generic_pid::set_ki(double _ki){
            ki=_ki;
        }
        void generic_pid::set_kd(double _kd){
            kd=_kd;
        }
        void generic_pid::set_bias(double _bias){
            bias=_bias;
        }
        void generic_pid::update_setpoint(double _setpoint){
            setpoint=_setpoint;
        }
        double generic_pid::calculate_output(double _input){
            auto e = setpoint - _input;
            auto output = kp * e + ki * e * T + (kd * e) / T;
            return output;
        }
        
    
}