#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_gnss_relative.hpp>

#include "karma_filter.hpp"


/*              还需要完成的地方
 * 在完成计算后，应该创建一个发布者将计算的导航量发布
*/

class KfGnssIns : public rclcpp::Node
{
public:
  explicit KfGnssIns() : Node("kf_gnss_ins")
  {

    // 配置通信质量
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    // 创建订阅对象
    imu_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
      "/fmu/out/sensor_combined", qos, 
      std::bind(&KfGnssIns::imuCallback, this, std::placeholders::_1));

    gnss_sub_ = this->create_subscription<px4_msgs::msg::SensorGnssRelative>(
      "/fmu/out/sensor_gnss_relative", qos, 
      std::bind(&KfGnssIns::gnssCallback, this, std::placeholders::_1));
  }
 
private:
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGnssRelative>::SharedPtr gnss_sub_;

  void imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  void gnssCallback(const px4_msgs::msg::SensorGnssRelative::SharedPtr msg);
  
  // 滤波器对象
  Filter filter_;
  
};


void KfGnssIns::imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg){
  // to-do

  // 更新IMU数据
  filter_.updateImu();
  // to-do
}
void kfGnssIns::gnssCallback(const px4_msgs::msg::SensorGnssRelative::SharedPtr msg){
  // to-do

  // 更新GNSS数据
  filter_.updateGnss();
  // 完成一次互补滤波
  filter_.fused_gnss_ins();

  // to-do
}









int main(int argc, char *argv[])
{
  std::cout << "Start Gnss Ins data fusion." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KfGnssIns>());
 
  rclcpp::shutdown();
  return 0;
}
 