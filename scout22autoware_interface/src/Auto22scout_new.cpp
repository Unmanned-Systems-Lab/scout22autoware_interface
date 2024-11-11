#include <autoware_control_msgs/msg/control.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <chrono> // 包含 chrono 库

float WHEEL_TREAD = 0.116;//scout的
float WHEEL_BASE = 0.498;

using namespace std::chrono_literals;

class Autoscoutpub : public rclcpp::Node
{
public:
    Autoscoutpub() : Node("Autoscoutpub"),count(0){
        RCLCPP_INFO(this->get_logger(),"PUB Node 启动");
        
        Autosub_ = this->create_subscription<autoware_control_msgs::msg::Control>("/control/command/control_cmd", 10, std::bind(&Autoscoutpub::callback_control_cmd, this, std::placeholders::_1));//订阅ctrl_cmd 来自autoware的指示
        
        Autopub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS{1});
        
        timer_ = this->create_wall_timer(0.01s,std::bind(&Autoscoutpub::on_timer,this));//定时器
    };

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Autopub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr Autosub_;
    
    // autoware command messages定义一些存放的信息
    autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_ptr_;
    //要发布的速度指令
    geometry_msgs::msg::Twist twister;

    //控制指令的回调函数
    void callback_control_cmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg){
        // RCLCPP_INFO(this->get_logger(),"收到 autoware_cmd");
        control_cmd_ptr_ = msg;

        twister.linear.x = control_cmd_ptr_->longitudinal.velocity;
        // double linear_spd = 0;
        // linear_spd = twister.linear.x;
        double steer_rate = control_cmd_ptr_->lateral.steering_tire_rotation_rate;

        twister.angular.z = steer_rate;
    }

    //发布器和定时器
    size_t count;
    void on_timer(){
    Autopub_->publish(twister);
  }

};

int main(int argc, char ** argv)
{
  //初始化客户端
  rclcpp::init(argc,argv);

  // 调用回旋函数
  rclcpp::spin(std::make_shared<Autoscoutpub>());
  // 释放资源
  rclcpp::shutdown();

  return 0;
}