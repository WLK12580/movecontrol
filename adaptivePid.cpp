#include <iostream>
#include <cmath>

using namespace std;

class AdaptivePID {
public:
    AdaptivePID(double kp, double ki, double kd, double beta, double gamma) :
        kp(kp), ki(ki), kd(kd), beta(beta), gamma(gamma), last_error(0), integral(0) {}

    double calculate(double setpoint, double pv, double dt) {
        double error = setpoint - pv;
        integral += error * dt;

        double derivative = (error - last_error) / dt;

        double output = kp * error + ki * integral + kd * derivative;

        double alpha = beta / (beta + abs(error));
        kp += gamma * alpha * error;
        ki += gamma * alpha * integral;
        kd += gamma * alpha * derivative;

        last_error = error;

        return output;
    }

private:
    double kp, ki, kd;
    double beta, gamma;
    double last_error;
    double integral;
};

int main() {
    AdaptivePID pid(1, 0.1, 0.5, 0.1, 0.01);

    double setpoint = 50;
    double pv = 0;
    double dt = 0.1;

    for (int i = 0; i < 100; i++) {
        double output = pid.calculate(setpoint, pv, dt);
        pv += output * dt;

        cout << "Setpoint: " << setpoint << ", PV: " << pv << ", Output: " << output << endl;
    }

    return 0;
}

