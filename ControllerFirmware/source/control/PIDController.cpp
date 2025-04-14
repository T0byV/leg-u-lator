#include "control/PIDController.h"
#include <cmath>

PIDController::PIDController(double Kp, double Ki, double Kd, 
                             double deadband, double baselinePower,
                             double sampleTime, double alphaTemp, double alphaDeriv,
                             double maxOutput)
    : Kp(Kp), Ki(Ki), Kd(Kd), deadband(deadband), baselinePower(baselinePower),
      sampleTime(sampleTime), alphaTemp(alphaTemp), alphaDeriv(alphaDeriv), maxOutput(maxOutput),
      integral(0.0), prevError(0.0), filteredTemp(0.0), filteredDerivative(0.0),
      firstRun(true)
{
}

double PIDController::update(double setpoint, double measured) {
    // Filter the measured temperature
    if (firstRun) {
        filteredTemp = measured;
        firstRun = false;
    } else {
        filteredTemp = alphaTemp * filteredTemp + (1.0 - alphaTemp) * measured;
    }
    
    double error = setpoint - filteredTemp;
    
    // Apply deadband
    if (std::fabs(error) < deadband) {
        error = 0.0;
    }
    
    integral += error * sampleTime;
    
    double rawDerivative = (error - prevError) / sampleTime;
    
    // Filter the derivative term
    filteredDerivative = alphaDeriv * filteredDerivative + (1.0 - alphaDeriv) * rawDerivative;
    
    // Compute the PID output 
    double pidOutput = Kp * error + Ki * integral + Kd * filteredDerivative;
    
    // Feed-forward baseline power
    double output = baselinePower + pidOutput;
    
    // Saturation
    if (output > maxOutput) {
        output = maxOutput;
        // Anti-windup
        integral -= error * sampleTime;
    }
    if (output < baselinePower) {
        output = baselinePower;
    }
    
    prevError = error;
    
    return output;
}

void PIDController::reset() {
    integral = 0.0;
    prevError = 0.0;
    filteredTemp = 0.0;
    filteredDerivative = 0.0;
    firstRun = true;
}
