#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <PidController.h>
#include <Kinematics.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>

Esp32McpwmMotor motor; // 创建一个名为motor的对象用于控制电机
Esp32PcntEncoder encoders[2]; // 创建一个数组用于储存两个编码器
PidController pid_controller[2]; // 创建PID控制器对象数组
Kinematics kinematics;

float target_linear_speed = 0.0; //目标线速度，单位：mm/s
float target_angular_speed = 0.0f; //目标角速度，单位：rad/s
float out_left_speed;
float out_right_speed;

// 声明相关的结构体对象
rcl_allocator_t allocator; //内存分配器，用于动态内存分配管理
rclc_support_t support;   //用于存储时钟、内存分配器和上下文，提供支持
rclc_executor_t executor; //执行器，用于管理订阅和计数器回调的执行
rcl_node_t node;          //节点

rcl_subscription_t subscriber; //创建订阅者
geometry_msgs__msg__Twist sub_msg; //储存订阅到的速度消息

rcl_publisher_t odom_publisher; //创建发布者
nav_msgs__msg__Odometry odom_msg; //储存里程计消息
rcl_timer_t timer; //定时器，用于定时调用函数

void twist_callback(const void *msg_in)
{
  //将接收到的消息指针转化为 geometry_msgs__msg__Twist 类型
  const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
  //运动学逆解并设置速度
  kinematics.kinematic_inverse(twist_msg->linear.x * 1000, twist_msg->angular.z, out_left_speed, out_right_speed);
  pid_controller[0].update_target(out_left_speed);
  pid_controller[1].update_target(out_right_speed);
}

// 在定时器回调函数中完成话题发布
void callback_publisher(rcl_timer_t *timer, int64_t last_call_time)
{
  odom_t odom = kinematics.get_odom();  //获取里程计
  int64_t stamp = rmw_uros_epoch_millis(); //获取当前时间
  odom_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000); //秒部分
  //纳秒部分
  odom_msg.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000 * 1e6);
  odom_msg.pose.pose.position.x = odom.x;
  odom_msg.pose.pose.position.y = odom.y;
  odom_msg.pose.pose.orientation.w = cos(odom.angle * 0.5);
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = sin(odom.angle * 0.5);
  odom_msg.twist.twist.angular.z = odom.angle_speed;
  odom_msg.twist.twist.linear.x = odom.linear_speed;
  //发布里程计
  if(rcl_publish(&odom_publisher, &odom_msg, NULL) != RCL_RET_OK)
  {
    Serial.printf("error: odom publisher failed!\n");
  }
}

// 单独创建一个任务运行 micro_ROS, 相当于一个线程
void micro_ros_task(void *parameter)
{
  //1.设置传输协议并延时等待设置完成
  IPAddress agent_ip;
  agent_ip.fromString("192.0.2.1");
  set_microros_wifi_transports("YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD", agent_ip, 8888);
  delay(2000);
  //2.初始化内存分配器
  allocator = rcl_get_default_allocator();
  //3.初始化 support
  rclc_support_init(&support, 0, NULL, &allocator);
  //4.初始化节点
  rclc_node_init_default(&node, "fishbot_motion_control", "", &support);
  //5.初始化执行器
  unsigned int num_handles = 0 + 2;
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  //6.初始化订阅者并添加到执行器中
  rclc_subscription_init_best_effort(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);
  //7.初始化发布者和定时器
  odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint");
  rclc_publisher_init_best_effort(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
  //8.时间同步
  while (!rmw_uros_epoch_synchronized())
  {
    rmw_uros_sync_session(1000);
    delay(10);
  }
  //9.创建定时器，间隔50ms发布调用一次callback_publisher
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), callback_publisher);
  rclc_executor_add_timer(&executor, &timer);
  //10.循环执行器
  rclc_executor_spin(&executor);
}

void setup() 
{
  Serial.begin(115200);
  // 初始化编码器
  encoders[0].init(0, 32, 33);
  encoders[1].init(1, 26, 25);
  // 初始化电动机
  motor.attachMotor(0, 22, 23); // 将电机0,接到引脚22和23
  motor.attachMotor(1, 12, 13);
  // 初始化PID控制器参数
  pid_controller[0].update_pid(0.625, 0.125, 0.0);
  pid_controller[1].update_pid(0.625, 0.125, 0.0);
  pid_controller[0].out_limit(-100, 100);
  pid_controller[1].out_limit(-100, 100);
  // 初始化轮距和电机参数
  kinematics.set_wheel_distance(175);
  kinematics.set_motor_param(0, 0.10658);
  kinematics.set_motor_param(1, 0.10658);
  // 运动学逆解并设置参数
  kinematics.kinematic_inverse(target_linear_speed, target_angular_speed, out_left_speed, out_right_speed);
  pid_controller[0].update_target(out_left_speed);
  pid_controller[1].update_target(out_right_speed);
  // 创建任务运行 micro_ros_task
  xTaskCreate(micro_ros_task, //任务函数
              "micro_ros",    //任务名称
              10240,          //任务堆栈大小（字节）
              NULL,           //传递给任务函数的参数
              1,              //任务优先级
              NULL            //任务句柄
              );
}

void loop() 
{
  Serial.printf("x=%f, y=%f, angle=%f\n", kinematics.get_odom().x, kinematics.get_odom().y, kinematics.get_odom().angle);
  delay(10); //ms
  kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks());
  motor.updateMotorSpeed(0, pid_controller[0].update(kinematics.get_motor_speed(0)));
  motor.updateMotorSpeed(1, pid_controller[1].update(kinematics.get_motor_speed(1)));
}
