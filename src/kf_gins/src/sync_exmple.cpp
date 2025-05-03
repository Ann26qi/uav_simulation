#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
// 根据消息类型调整
#include <px4_msgs/msg/image.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using LaserScanMsg = sensor_msgs::msg::LaserScan;

// 近似时间策略
typedef message_filters::sync_policies::ApproximateTime<ImageMsg, LaserScanMsg> ApproximatePolicy;

// 严格时间策略
// typedef message_filters::sync_policies::ExactTime<ImageMsg, LaserScanMsg> ExactPolicy;


class SyncNode : public rclcpp::Node {
public:
  SyncNode() : Node("time_sync_node") {
    // 创建订阅者
    image_sub_.subscribe(this, "/camera/image");
    scan_sub_.subscribe(this, "/lidar/scan");

    // 初始化同步器（以ApproximateTime为例）
    sync_ = std::make_shared<message_filters::Synchronizer<ApproximatePolicy>>(
      ApproximatePolicy(10), image_sub_, scan_sub_);
    
    // 注册同步回调函数
    sync_->registerCallback(&SyncNode::sync_callback, this);
  }

private:
  message_filters::Subscriber<ImageMsg> image_sub_;
  message_filters::Subscriber<LaserScanMsg> scan_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproximatePolicy>> sync_;
};

void sync_callback(const ImageMsg::ConstSharedPtr& image,
                   const LaserScanMsg::ConstSharedPtr& scan) {
  // 检查消息有效性
  if (!image || !scan) {
    RCLCPP_WARN(this->get_logger(), "Received null message");
    return;
  }

  // 获取消息时间戳
  rclcpp::Time image_time = image->header.stamp;
  rclcpp::Time scan_time = scan->header.stamp;
  
  // 处理同步后的数据
  RCLCPP_INFO(this->get_logger(), 
              "Synced messages: Image[%s] Scan[%s]",
              std::to_string(image_time.nanoseconds()).c_str(),
              std::to_string(scan_time.nanoseconds()).c_str());
  
  // 在此添加具体处理逻辑...
}
