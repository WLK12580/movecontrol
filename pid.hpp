#pragma once
#include <iostream>

namespace PidController {
class PID {
 public:
  PID() = default;
  PID(float kp, float ki, float kd);

  float update(float control);  //更新输出值
  void reset(); //重置pid
  void update_pid(float kp, float ki, float kd); //更新pid参数
  void update_target_(float target); //更新目标值
  void out_limit(float min, float max); //输出限制

  void setTargetValue(float targetValue){
    target_value_=targetValue;
  }
 private:
  float target_value_;
  float out_min_;
  float out_max_;
  float kp_;
  float ki_;
  float kd_;
  float last_output_value_;

  float error_sum_;
  float derror_;
  float error_pre_;
  float error_last_;
  float internal_up_ = 2500;
};
}  // namespace PidController