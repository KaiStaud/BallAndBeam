#include <stdint.h>

namespace controllib{

    class generic_pid{
        private: 
        
        double last_sample;
        double kp; 
        double ki;
        double kd;
        double bias;
        double setpoint;
        double T;

        public:
        generic_pid(double _kp, double _ki, double _kd, double _bias,double _T);
        void set_kp(double _kp);
        void set_ki(double _ki);
        void set_kd(double _kd);
        void set_bias(double _bias);
        void update_setpoint(double _setpoint);
        double calculate_output(double _input);
        
    };
}