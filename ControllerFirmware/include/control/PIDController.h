#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, 
                  double deadband, double baselinePower,
                  double sampleTime, double alphaTemp, double alphaDeriv,
                  double maxOutput);

    double update(double setpoint, double measured);

    void reset();

private:
    // PID gains
    double Kp, Ki, Kd;
    
    // Deadband [K]
    double deadband;
    
    // Feed-forward baseline power [W]
    double baselinePower;
    
    // Sampling time [s]
    double sampleTime;
    
    // Filtering coefficients (0 < alpha < 1)
    double alphaTemp;
    double alphaDeriv;
    
    // Maximum output power [W]
    double maxOutput;

    // Internal state variables
    double integral;
    double prevError;
    double filteredTemp;
    double filteredDerivative;
    bool firstRun;
};

#endif 
