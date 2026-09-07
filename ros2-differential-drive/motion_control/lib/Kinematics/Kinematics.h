#ifndef __KINEMATICS_H__ //防止头文件被多次包含
#define __KINEMATICS_H__

#include<Arduino.h>

// 定义一个结构用于储存电机参数
typedef struct
{
    float per_pulse_distance; //单个脉冲对应轮子的前进距离
    int16_t motor_speed; //当前电机的速度 mm/s
    int64_t last_encoder_tick; //赏赐电动机编码读数
} motor_param_t;

// 定义一个结构用于储存里程计数据
typedef struct 
{
    float x; //x坐标
    float y; //y坐标
    float angle; //角度
    float linear_speed; //线速度
    float angle_speed; //角速度
}odom_t;

// 定义一个类用于处理机器人运动学
class Kinematics
{
    public:
        Kinematics() = default; //构造函数
        ~Kinematics() = default; //析构函数，用于销毁对象
        void set_motor_param(uint8_t id, float per_pulse_distance); //设置电机参数id，palus/mm
        void set_wheel_distance(float wheel_distance); //设置轮子间距
        // IK将期望线速度和角速度转化为左右轮速度
        void kinematic_inverse(float linear_speed, float angle_speed,
                               float &out_left_speed, float &out_right_speed);
        // FK将左右轮速度转换为线速度和角速度
        void kinematic_forward(float left_speed, float right_speed,
                               float &out_linear_speed, float &out_angle_speed);
        // 更新电动机速度和编码器数据
        void update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick);
        // 获取电机速度
        int16_t get_motor_speed(uint8_t id);
        // 更新里程计数据
        void update_odom(uint16_t dt);
        // 获取里程计数据
        odom_t &get_odom();
        // 用于将角度转换到正负180度
        static void TransAngleInPI(float angle, float &out_angle);

    private:
        motor_param_t motor_param_[2]; //储存两个电机的参数
        uint64_t last_update_time; //上次更新数据的时间，ms
        float wheel_distance_; //轮子间距
        odom_t odom_; //储存里程计信息
};

#endif //__KINEMATICS_H__