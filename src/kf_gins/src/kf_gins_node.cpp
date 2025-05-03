#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_gnss_relative.hpp>

#include "karma_filter.hpp"


class KfGnssIns : public rclcpp::Node
{
public:
  explicit KfGnssIns() : Node("kf_gnss_ins")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
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

  Filter filter_;
  
};


void KfGnssIns::imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg){
  // to-do
  filter_.updateImu();
  // to-do
}
void kfGnssIns::gnssCallback(const px4_msgs::msg::SensorGnssRelative::SharedPtr msg){
  // to-do
  filter_.updateGnss();
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
 