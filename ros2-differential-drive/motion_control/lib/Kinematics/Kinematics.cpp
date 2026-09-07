#include "Kinematics.h"

// 设置电机参数
void Kinematics::set_motor_param(uint8_t id, float per_pulse_distance)
{
    motor_param_[id].per_pulse_distance = per_pulse_distance; //每个电机单个脉冲前进的距离
}

// 设置小车轮距
void Kinematics::set_wheel_distance(float wheel_distance)
{
    wheel_distance_ = wheel_distance;
}

// 获取电机速度
int16_t Kinematics::get_motor_speed(uint8_t id)
{
    return motor_param_[id].motor_speed;
}

// 更新电动机速度
void Kinematics::update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick)
{
    //计算dt
    uint32_t dt = current_time - last_update_time;
    last_update_time = current_time;
    //计算电动机的编码器读数变化量
    int32_t dtick1 = left_tick - motor_param_[0].last_encoder_tick;
    int32_t dtick2 = right_tick - motor_param_[1].last_encoder_tick;
    motor_param_[0].last_encoder_tick = left_tick;
    motor_param_[1].last_encoder_tick = right_tick;
    //计算轮子速度
    motor_param_[0].motor_speed = float(dtick1 * motor_param_[0].per_pulse_distance) / dt * 1000; //mm/s
    motor_param_[1].motor_speed = float(dtick2 * motor_param_[1].per_pulse_distance) / dt * 1000;
    //更新里程计数据
    update_odom(dt);
}

// 正运动学计算
void Kinematics::kinematic_forward(float left_speed, float right_speed,
                                    float &out_linear_speed,
                                    float &out_angle_speed)
{
    //两轮转速之和除以2
    out_linear_speed = (right_speed + left_speed) / 2.0;
    //两轮转速之差除以轮距
    out_angle_speed = (right_speed - left_speed) / wheel_distance_;
}

// 逆运动学计算
void Kinematics::kinematic_inverse(float linear_speed, float angle_speed,
                                float &out_left_speed, float &out_right_speed)
{
    out_left_speed = linear_speed - (angle_speed * wheel_distance_) / 2.0;
    out_right_speed = linear_speed + (angle_speed * wheel_distance_) / 2.0;
}

// 获取里程计数据
odom_t &Kinematics::get_odom() { return odom_; };

// 用于将角度转换到正负180度
void Kinematics::TransAngleInPI(float angle, float &out_angle)
{
    if (angle > PI)
    {
        out_angle -= 2 * PI;
    }
    else if (angle < -PI)
    {
        out_angle += 2 * PI;
    }
}

// 更新里程计
void Kinematics::update_odom(uint16_t dt)
{
    //ms -> s
    float dt_s = (float)dt / 1000;
    //运动学正解，计算线速度和角速度
    this->kinematic_forward(motor_param_[0].motor_speed, motor_param_[1].motor_speed, 
                            odom_.linear_speed, odom_.angle_speed);
    //转换线速度单位（mm/s -> m/s)
    odom_.linear_speed = odom_.linear_speed / 1000;
    //计算当前角度 
    odom_.angle += odom_.angle_speed * dt_s;
    //将角度值转换到正负180度
    Kinematics::TransAngleInPI(odom_.angle, odom_.angle);
    //计算机器人移动距离和在两轴上的分量并进行累积
    float delta_distance = odom_.linear_speed * dt_s;
    odom_.x += delta_distance * std::cos(odom_.angle);
    odom_.y += delta_distance * std::sin(odom_.angle);
}