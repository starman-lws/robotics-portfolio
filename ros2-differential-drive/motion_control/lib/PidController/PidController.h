#ifndef __PIDCONTROLLER_H__ //防止头文件被多次包含
#define __PIDCONTROLLER_H__

class PidController
{
    public:
        PidController() = default; //默认构造函数，kp、ki、kd使用默认值
        PidController(float kp, float ki, float kd); //构造函数

    private:
        float target_;  //目标值
        float out_min_; //输出下限
        float out_max_; //输出上限
        float kp_;      //比例系数
        float ki_;      //积分系数
        float kd_;      //微分系数

        float error_sum_;           //误差累积和
        float derror_;              //误差变化率
        float error_last_;          //上一次误差
        float error_pre_;           //上上次误差
        float intergral_up_ = 2500; //积分上限

    public:
        float update(float current);                   //提供当前值返回下次输出值
        void update_target(float target);              //更新目标值
        void update_pid(float kp, float ki, float kd); //更新PID系数
        void reset();                                  //重置PID控制器
        void out_limit(float out_min, float out_max);  //设置输出限制
};

#endif // __PIDCONTROLLER_H__ 