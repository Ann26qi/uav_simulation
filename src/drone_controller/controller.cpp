#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

using namespace std::chrono_literals;

class MissionControlNode : public rclcpp::Node {
public:
    MissionControlNode() : Node("mission_control_node") {
        // 初始化客户端和服务
        wp_push_client_ = this->create_client<mavros_msgs::srv::WaypointPush>("mavros/mission/push");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        
        // 定义航点（示例为本地坐标系下的NED位置）
        define_waypoints();
        
        // 等待服务可用
        while (!wp_push_client_->wait_for_service(1s) || !set_mode_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "等待服务...");
        }
        
        // 上传航点并执行任务
        upload_waypoints();
        set_mission_mode();
    }

private:
    void define_waypoints() {
        // 航点定义（NED坐标系，z向下为负）
        waypoints_.push_back(create_waypoint(0.0, 0.0, -5.0));  // 起飞点
        waypoints_.push_back(create_waypoint(5.0, 0.0, -5.0));  // 第一个航点
        waypoints_.push_back(create_waypoint(5.0, 5.0, -5.0));  // 第二个航点
        waypoints_.push_back(create_waypoint(0.0, 5.0, -5.0));  // 返回
    }

    mavros_msgs::msg::Waypoint create_waypoint(float x, float y, float z) {
        mavros_msgs::msg::Waypoint wp;
        wp.frame = mavros_msgs::msg::Waypoint::FRAME_LOCAL_NED;
        wp.command = 16;  // MAV_CMD_NAV_WAYPOINT
        wp.x_lat = x;
        wp.y_long = y;
        wp.z_alt = z;
        wp.autocontinue = true;
        wp.is_current = false;
        return wp;
    }

    void upload_waypoints() {
        auto request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
        request->waypoints = waypoints_;
        
        auto future = wp_push_client_->async_send_request(request);
        if (future.wait_for(5s) == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "航点上传成功，共 %zu 个航点", response->wp_transfered);
            } else {
                RCLCPP_ERROR(this->get_logger(), "航点上传失败");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
        }
    }

    void set_mission_mode() {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "AUTO.MISSION";
        
        auto future = set_mode_client_->async_send_request(request);
        if (future.wait_for(5s) == std::future_status::ready) {
            auto response = future.get();
            if (response->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "已切换至Mission模式");
            } else {
                RCLCPP_ERROR(this->get_logger(), "模式切换失败");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
        }
    }

    // 成员变量
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr wp_push_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    std::vector<mavros_msgs::msg::Waypoint> waypoints_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}