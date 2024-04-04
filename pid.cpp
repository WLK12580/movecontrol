#include "pid.hpp"

using namespace PidController;

PID::PID(float kp, float ki, float kd) {
  reset();
  update_pid(kp, ki, kd);
}

float PID::update(float control) {
  float error = target_value_ - control;  // 计算误差
  
  derror_ = error_last_ - error;
  error_last_ = error;
  std::cout<<"error="<<error<<" targetValue="<<target_value_<<" derror="<<derror_<<std::endl;
  error_sum_ += error;
  if (error_sum_ > internal_up_) {
    error_sum_ = internal_up_;
  }
  if (error_sum_ < -internal_up_) {
    error_sum_ = -internal_up_;
  }
  // 计算pid
  float output = kp_ * error + ki_ * error_sum_ + kd_ * derror_;
  if (output > out_max_) {
    output = out_max_;
  }
  if (output < out_min_) {
    output = out_min_;
  }
  last_output_value_ = output;
  return last_output_value_;
}
void PID::update_pid(float kp, float ki, float kd)
{
    reset(); // 重置控制器状态
    kp_ = kp; // 更新比例项系数
    ki_ = ki; // 更新积分项系数
    kd_ = kd; // 更新微分项系数
}
void PID::reset()
{
    // 重置控制器状态
    last_output_value_ = 0.0f; // 上一次的控制输出值
    target_value_ = 0.0f; // 控制目标值
    out_min_ = 0.0f; // 控制输出最小值
    out_max_ = 0.0f; // 控制输出最大值
    kp_ = 0.0f; // 比例项系数
    ki_ = 0.0f; // 积分项系数
    kd_ = 0.0f; // 微分项系数
    error_sum_ = 0.0f; // 误差累计值
    derror_ = 0.0f; // 误差变化率
    error_last_ = 0.0f; // 上一次的误差值
}

void PID::out_limit(float out_mix, float out_max)
{
    out_min_ = out_mix; // 控制输出最小值
    out_max_ = out_max; // 控制输出最大值
}
