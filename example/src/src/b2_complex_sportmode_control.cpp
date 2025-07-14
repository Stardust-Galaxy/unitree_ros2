
#include <memory>
#include <string>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class B2ComplexTrajectoryControl : public rclcpp::Node {
public:
    B2ComplexTrajectoryControl() : Node("b2_complex_trajectory_control") {
        // 声明参数
        this->declare_parameter("max_linear_velocity", 0.8);
        this->declare_parameter("max_lateral_velocity", 0.6);
        this->declare_parameter("max_angular_velocity", 0.8);
        this->declare_parameter("control_frequency", 50.0);
        this->declare_parameter("trajectory_mode", "figure8"); // figure8, circle, zigzag, spiral
        this->declare_parameter("zigzag_forward_speed", 0.2);
        this->declare_parameter("zigzag_lateral_amplitude", 0.5);
        this->declare_parameter("zigzag_angular_amplitude", 0.6);
        this->declare_parameter("zigzag_period", 3.0);

        max_vx_ = this->get_parameter("max_linear_velocity").as_double();
        max_vy_ = this->get_parameter("max_lateral_velocity").as_double();
        max_vyaw_ = this->get_parameter("max_angular_velocity").as_double();
        control_freq_ = this->get_parameter("control_frequency").as_double();
        trajectory_mode_ = this->get_parameter("trajectory_mode").as_string();
        
        // 之字形轨迹参数
        zigzag_forward_speed_ = this->get_parameter("zigzag_forward_speed").as_double();
        zigzag_lateral_amplitude_ = this->get_parameter("zigzag_lateral_amplitude").as_double();
        zigzag_angular_amplitude_ = this->get_parameter("zigzag_angular_amplitude").as_double();
        zigzag_period_ = this->get_parameter("zigzag_period").as_double();

        // 发布运动请求
        req_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // 定时器，定期下发运动指令
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq_)),
            std::bind(&B2ComplexTrajectoryControl::timerCallback, this));

        // 初始化时间
        start_time_ = this->now();
        last_time_ = start_time_;

        RCLCPP_INFO(this->get_logger(), "B2 Complex Trajectory Control Node started.");
        RCLCPP_INFO(this->get_logger(), "Trajectory mode: %s", trajectory_mode_.c_str());
    }

private:
    void timerCallback() {
        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds();
        
        // 计算轨迹速度
        calculateTrajectoryVelocity(elapsed_time);
        
        // 构造运动请求
        unitree_api::msg::Request req;
        sport_client_.Move(req, vx_, vy_, vyaw_);
        req_pub_->publish(req);
        
        // 每2秒打印一次状态
        if ((current_time - last_time_).seconds() > 2.0) {
            RCLCPP_INFO(this->get_logger(), 
                "Time: %.1fs, Vx: %.2f, Vy: %.2f, Vyaw: %.2f", 
                elapsed_time, vx_, vy_, vyaw_);
            last_time_ = current_time;
        }
    }

    void calculateTrajectoryVelocity(double time) {
        if (trajectory_mode_ == "figure8") {
            calculateFigure8Trajectory(time);
        } else if (trajectory_mode_ == "circle") {
            calculateCircleTrajectory(time);
        } else if (trajectory_mode_ == "zigzag") {
            calculateZigzagTrajectory(time);
        } else if (trajectory_mode_ == "spiral") {
            calculateSpiralTrajectory(time);
        } else {
            // 默认8字形轨迹
            calculateFigure8Trajectory(time);
        }
    }

    void calculateFigure8Trajectory(double time) {
        // 8字形轨迹 - 适中的动作幅度
        double period = 12.0; // 12秒一个周期
        double t = fmod(time, period);
        double phase = 2.0 * M_PI * t / period;
        
        // 8字形的参数方程
        double scale_x = 0.6;  // 前进后退幅度
        double scale_y = 0.4;  // 左右移动幅度
        double scale_yaw = 0.5; // 转向幅度
        
        // 计算速度分量
        vx_ = scale_x * cos(phase) * (1.0 + 0.3 * cos(2.0 * phase));
        vy_ = scale_y * sin(2.0 * phase);
        vyaw_ = scale_yaw * sin(phase) * cos(phase);
        
        // 限幅
        limitVelocities();
    }

    void calculateCircleTrajectory(double time) {
        // 圆形轨迹 - 原地旋转
        double period = 8.0; // 8秒一个周期
        double t = fmod(time, period);
        double phase = 2.0 * M_PI * t / period;
        
        double radius = 0.3; // 圆形半径（速度）
        double angular_speed = 0.6; // 角速度
        
        vx_ = radius * cos(phase);
        vy_ = radius * sin(phase);
        vyaw_ = angular_speed;
        
        limitVelocities();
    }

    void calculateZigzagTrajectory(double time) {
        // 之字形轨迹 - 前进时左右摆动
        double t = fmod(time, zigzag_period_);
        double phase = 2.0 * M_PI * t / zigzag_period_;
        
        // 前进速度
        vx_ = zigzag_forward_speed_;
        
        // 左右摆动
        vy_ = zigzag_lateral_amplitude_ * sin(4.0 * phase);
        
        // 转向配合摆动
        vyaw_ = zigzag_angular_amplitude_ * sin(4.0 * phase);
        
        limitVelocities();
    }

    void calculateSpiralTrajectory(double time) {
        // 螺旋轨迹 - 逐渐扩大或缩小
        double period = 10.0; // 10秒一个周期
        double t = fmod(time, period);
        double phase = 2.0 * M_PI * t / period;
        
        // 螺旋参数
        double base_radius = 0.2;
        double expansion_rate = 0.1;
        double spiral_factor = sin(t / period * M_PI); // 0到1的变化
        
        double current_radius = base_radius + expansion_rate * spiral_factor;
        
        vx_ = current_radius * cos(phase);
        vy_ = current_radius * sin(phase);
        vyaw_ = 0.4 * (1.0 + 0.5 * sin(phase));
        
        limitVelocities();
    }

    void limitVelocities() {
        // 自定义clamp函数，兼容C++14
        vx_ = std::max(-max_vx_, std::min(vx_, max_vx_));
        vy_ = std::max(-max_vy_, std::min(vy_, max_vy_));
        vyaw_ = std::max(-max_vyaw_, std::min(vyaw_, max_vyaw_));
    }

    // ROS接口
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 运动参数
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vyaw_ = 0.0;
    double max_vx_ = 0.8;
    double max_vy_ = 0.6;
    double max_vyaw_ = 0.8;
    double control_freq_ = 50.0;
    std::string trajectory_mode_;
    
    // 之字形轨迹参数
    double zigzag_forward_speed_ = 0.2;
    double zigzag_lateral_amplitude_ = 0.5;
    double zigzag_angular_amplitude_ = 0.6;
    double zigzag_period_ = 3.0;

    // 时间相关
    rclcpp::Time start_time_;
    rclcpp::Time last_time_;

    // Unitree运动客户端
    SportClient sport_client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<B2ComplexTrajectoryControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
