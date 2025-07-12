#include <memory>
#include <string>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class B2SportModeControl : public rclcpp::Node {
public:
    B2SportModeControl() : Node("b2_sportmode_control") {
        // 声明参数
        this->declare_parameter("max_linear_velocity", 1.5);
        this->declare_parameter("max_lateral_velocity", 1.0);
        this->declare_parameter("max_angular_velocity", 1.0);
        this->declare_parameter("control_frequency", 50.0);

        max_vx_ = this->get_parameter("max_linear_velocity").as_double();
        max_vy_ = this->get_parameter("max_lateral_velocity").as_double();
        max_vyaw_ = this->get_parameter("max_angular_velocity").as_double();
        control_freq_ = this->get_parameter("control_frequency").as_double();

        // 订阅速度指令
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&B2SportModeControl::cmdVelCallback, this, std::placeholders::_1));

        // 发布运动请求
        req_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // 定时器，定期下发运动指令
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq_)),
            std::bind(&B2SportModeControl::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "B2 SportMode Velocity Control Node started.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 限幅
        vx_ = std::clamp(msg->linear.x, -max_vx_, max_vx_);
        vy_ = std::clamp(msg->linear.y, -max_vy_, max_vy_);
        vyaw_ = std::clamp(msg->angular.z, -max_vyaw_, max_vyaw_);
    }

    void timerCallback() {
        // 构造运动请求
        unitree_api::msg::Request req;
        sport_client_.Move(req, vx_, vy_, vyaw_);
        req_pub_->publish(req);
        RCLCPP_INFO(this->get_logger(), "Publishing velocity");
    }

    // ROS接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 运动参数
    double vx_ = -0.1;
    double vy_ = 0.0;
    double vyaw_ = 0.0;
    double max_vx_ = 1.5;
    double max_vy_ = 1.0;
    double max_vyaw_ = 1.0;
    double control_freq_ = 50.0;

    // Unitree运动客户端
    SportClient sport_client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<B2SportModeControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 