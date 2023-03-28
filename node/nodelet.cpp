// #include "manager/node_manager.hpp"
#include "source/source_driver.hpp"
#include "source/source_pointcloud_ros.hpp"
#include "msg/rs_msg/lidar_point_cloud_msg.hpp"

#include "rs_driver/api/lidar_driver.hpp"
#include <rs_driver/utility/sync_queue.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robosense::lidar
{

class RSLidarDriver: public rclcpp::Node 
{
public:
  RSLidarDriver(const rclcpp::NodeOptions & options)
  : Node("rslidar_driver_node", options)
  {
    init();
  }

  ~RSLidarDriver()
  {
    stop();
  }
private:
  virtual void init(void);
  virtual void stop(void);
  void sendPointCloud(const LidarPointCloudMsg& msg);
  void exceptionCallback(const Error& code);
  std::shared_ptr<LidarPointCloudMsg> pointCloudGetCallback(void);

  LidarDriver<LidarPointCloudMsg> driver_; ///< driver implementation class
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  std::string frame_id_;
  bool send_by_rows_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_point_cloud_queue_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> point_cloud_queue_;
};

void RSLidarDriver::init()
{
    // Declare parameters
    this->declare_parameter("lidar_type", "RS16");
    this->declare_parameter("input_type", "online");
    this->declare_parameter("msop_port", 6699);
    this->declare_parameter("difop_port", 7788);
    this->declare_parameter("point_cloud_topic", "rslidar_points");

    this->declare_parameter("send_by_rows", false);
    this->declare_parameter("dense_points", false);
    this->declare_parameter("frame_id", "rslidar");
    
    // ... declare other parameters as needed

    // Get parameters
    std::string lidar_type = this->get_parameter("lidar_type").as_string();
    std::string input_type = this->get_parameter("input_type").as_string();
    int msop_port = this->get_parameter("msop_port").as_int();
    int difop_port = this->get_parameter("difop_port").as_int();
    std::string point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
    bool send_by_rows_ = this->get_parameter("send_by_rows").as_bool(); 
    bool dense_points = this->get_parameter("dense_points").as_bool(); 
    std::string frame_id_ = this->get_parameter("frame_id").as_string(); 
    // ... get other parameters as needed

    // Set driver parameters based on the values from the launch file
    RSDriverParam param;
    param.lidar_type = strToLidarType(lidar_type);
    param.input_type = (input_type == "online") ? InputType::ONLINE_LIDAR : InputType::PCAP_FILE;
    param.input_param.msop_port = msop_port;
    param.input_param.difop_port = difop_port;

    if (dense_points) 
    {
      send_by_rows_ = false;
    };
    // ... set other parameters as needed
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic, rclcpp::SensorDataQoS());

    // Initialize the driver with the parameters
    driver_.regExceptionCallback(std::bind(&RSLidarDriver::exceptionCallback, this, std::placeholders::_1));
    driver_.regPointCloudCallback(std::bind(&RSLidarDriver::pointCloudGetCallback, this),
     std::bind(&RSLidarDriver::sendPointCloud, this, std::placeholders::_1));

    if (!driver_.init(param))
    {
      RCLCPP_ERROR(this->get_logger(), "Driver Initialize Error...");
      throw std::runtime_error("Driver Initialize Error");
    }

    // Start the driver
    driver_.start();
}

void RSLidarDriver::stop()
{
  driver_.stop();
}

inline std::shared_ptr<LidarPointCloudMsg> RSLidarDriver::pointCloudGetCallback(void)
{
  std::shared_ptr<LidarPointCloudMsg> point_cloud = free_point_cloud_queue_.pop();
  if (point_cloud.get() != NULL)
  {
    return point_cloud;
  }

  return std::make_shared<LidarPointCloudMsg>();
}

inline void RSLidarDriver::sendPointCloud(const LidarPointCloudMsg& msg)
{
  point_cloud_pub_->publish(toRosMsg(msg, frame_id_, send_by_rows_));
}

void RSLidarDriver::exceptionCallback(const Error& code)
{
  code.toString();
  // RCLCPP_WARN(this->get_logger(), code.toString());
}

std::shared_ptr<LidarPointCloudMsg> RSLidarDriver::pointCloudGetCallback(void)
{
  std::shared_ptr<LidarPointCloudMsg> msg = free_cloud_queue_.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<LidarPointCloudMsg>();
}

inline std::shared_ptr<LidarPointCloudMsg> RSLidarDriver::getPointCloud(void)
{
  std::shared_ptr<LidarPointCloudMsg> point_cloud = free_point_cloud_queue_.pop();
  if (point_cloud.get() != NULL)
  {
    return point_cloud;
  }

  return std::make_shared<LidarPointCloudMsg>();
}

} // namespace robosense::lidar

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RSLidarDriver)