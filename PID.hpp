namespace pid_controller
{
    class PID
    {
        public:
            PID(double kp, double kd, double ki, double time_interval);
            double compute_adjustment(double current_error);
            double get_error();

        private:
            double KP;
            double KD;
            double KI;
            double time_interval;
            double prev_error;
            double derivation_error;
            double integral_error;
    };
}