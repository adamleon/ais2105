//
// Created by ein on 25.11.24.
//
#include <iostream>
#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    // Constructor to initialize PID coefficients and reference value
    PIDController(double kp, double ki, double kd)
        : p(kp), i(ki), d(kd), reference(0.0), effort(0.0), prevError(0.0), integral(0.0) {}

    // Function to update the effort based on the measured value
    void update(double measuredValue) {
        double error = reference - measuredValue; // Calculate the error
        integral += error; // Calculate the integral term
        double derivative = error - prevError; // Calculate the derivative term
        
        // Calculate the control effort
        effort = p * error + i * integral + d * derivative;
        
        // Update previous error for the next iteration
        prevError = error;
    }

    // Getter for the effort value
    double getEffort() const {
        return effort;
    }

    // Setter for the reference value
    void setReference(double ref) {
        reference = ref;
    }

private:
    double p; // Proportional coefficient
    double i; // Integral coefficient
    double d; // Derivative coefficient
    double reference; // Desired reference value
    double effort; // Control effort
    double prevError; // Previous error value
    double integral; // Integral of the error over time
};

#endif //PIDCONTROLLER_H
