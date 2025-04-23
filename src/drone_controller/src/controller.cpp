#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class TrajectoryNode : public rclcpp::Node {
public:
    TrajectoryNode() : Node("trajectory_node"), current_waypoint_(0) {
        // 初始化发布器
        setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // 定义航点（NED坐标系：下为负）
        waypoints_ = {
            {0.0, 0.0, -5.0},
            {5.0, 0.0, -5.0},
            {5.0, 5.0, -5.0},
            {0.0, 5.0, -5.0},
            {0.0, 0.0, -5.0}
        };

        // 定时器更新目标点（2秒间隔）
        timer_ = this->create_wall_timer(
            10000ms, std::bind(&TrajectoryNode::update_waypoint, this));

        // 切换到Offboard模式
        switch_to_offboard();
    }

private:
    void switch_to_offboard() {
        auto cmd = px4_msgs::msg::VehicleCommand();
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        cmd.target_system = 1;
        cmd.target_component = 1;
        command_publisher_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "切换到Offboard模式");
    }

    void update_waypoint() {
        if (current_waypoint_ >= waypoints_.size()) {
            current_waypoint_ = 0;
        }

        auto setpoint = px4_msgs::msg::TrajectorySetpoint();
        setpoint.position[0] = waypoints_[current_waypoint_][0];
        setpoint.position[1] = waypoints_[current_waypoint_][1];
        setpoint.position[2] = waypoints_[current_waypoint_][2];
        setpoint.yaw = 0.0;  // 偏航角

        setpoint_publisher_->publish(setpoint);
        RCLCPP_INFO(this->get_logger(), "发布目标点: [%.1f, %.1f, %.1f]", 
            waypoints_[current_waypoint_][0], 
            waypoints_[current_waypoint_][1], 
            waypoints_[current_waypoint_][2]);

        current_waypoint_++;
    }

    // 成员变量
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::array<float, 3>> waypoints_;
    size_t current_waypoint_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}