#include "PidController.h"
#include "Arduino.h"

PidController::PidController(float kp, float ki, float kd)
{
    reset(); //初始化PID控制器
    update_pid(kp,ki,kd);
}

float PidController::update(float current)
{
    //计算误差及变化率
    float error = target_ - current; //计算误差
    derror_ = error_last_ - error; //计算误差变化率
    error_last_ = error; //更新上一次误差为当前误差

    //计算积分项并进行积分限制
    error_sum_ += error;
    if (error_sum_ > intergral_up_)
        error_sum_ = intergral_up_;
    if (error_sum_ < -1 * intergral_up_)
        error_sum_ = -1 * intergral_up_;
    
    //计算控制输出值
    float output = kp_ * error + ki_ * error_sum_ + kd_ * derror_;

    //控制输出限幅
    if (output > out_max_)
        output = out_max_;
    if (output < out_min_)
        output = out_min_;

    return output;
}

void PidController::update_target(float target)
{
    target_ = target; //更新控制目标
}

void PidController::update_pid(float kp, float ki, float kd)
{
    reset();  //重置控制器状态
    kp_ = kp; 
    ki_ = ki;
    kd_ = kd;
}

void PidController::reset()
{
    target_ = 0.0f;
    out_min_ = 0.0f;
    out_max_ = 0.0f;
    kp_ = 0.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;
    error_sum_ = 0.0f;
    derror_ = 0.0f;
    error_last_ = 0.0f;
}

void PidController::out_limit(float out_min, float out_max)
{
    out_min_ = out_min;
    out_max_ = out_max;
}