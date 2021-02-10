//Use to create utility classes that share across all robot.
#pragma once
#include <ctime>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>

namespace RoverRobotics {
struct PidGains {
  double Kp;
  double Ki;
  double Kd;
  PidGains();
  PidGains(float p, float i, float d) : Kp(p), Ki(i), Kd(d) {}
};
class OdomControl {
 public:
  OdomControl();  // default

  OdomControl(bool use_control, PidGains pid_gains, int max, int min,
              std::ofstream* fs);  // max min values for returned value
  OdomControl(bool use_control, PidGains pid_gains, int max,
              int min, int neutral);  // max min values for returned value

  unsigned char run(double commanded_vel,
                    double measured_vel, double dt,
                    int firmwareBuildNumber);  // in m/s
  void reset();

  int MOTOR_MAX_;       // 250
  int MOTOR_MIN_;       // 0
  int MOTOR_DEADBAND_;  // = 9;
  int MOTOR_NEUTRAL_;
  double MAX_ACCEL_CUTOFF_;  // 20
  double MIN_VELOCITY_;      // 0.04
  double MAX_VELOCITY_;      // 2.5ish?

  bool use_control_;

  // Can poll these values to see if motor speed is saturating
  bool at_max_motor_speed_;
  bool at_min_motor_speed_;
  bool stop_integrating_;

  //.csv Debuggin
  std::ofstream* fs_;

  // General Class variables
  double K_P_;
  double K_I_;
  double K_D_;
  double integral_error_;
  double differential_error_;
  double velocity_error_;

  // Returned value
  int motor_command_; 
  unsigned char deadband_offset_;

  // velocity feedback
  double velocity_commanded_;
  double velocity_measured_;
  double velocity_filtered_;
  std::vector<double> velocity_filtered_history_;
  std::vector<double> velocity_history_;
  bool velocity_control_on_;
  bool skip_measurement_;

 private:
  void velocityController();
  double filter(double left_motor_vel, double dt, int firmwareBuildNumber);
  bool hasZeroHistory(const std::vector<double>& vel_history);
  int boundMotorSpeed(int motor_speed, int max, int min);
  int deadbandOffset(int motor_speed, int deadband_offset);
  double P(double error);
  double I(double error, double dt);
  double D(double error, double dt);
  int PID(double error, double dt);
  int feedThroughControl();
};
}  // namespace RoverRobotics