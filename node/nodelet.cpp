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
  : Node("rslidar_driver_node", options), 
  to_exit_process_(false)
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
  std::shared_ptr<LidarPointCloudMsg> getPointCloud(void);
  void putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);
  void processPointCloud();

  std::shared_ptr<LidarDriver<LidarPointCloudMsg>> driver_ptr_; ///< driver implementation class
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  std::string frame_id_;
  bool send_by_rows_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_point_cloud_queue_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> point_cloud_queue_;

  std::thread point_cloud_process_thread_;
  bool to_exit_process_;
};

void RSLidarDriver::init()
{
    // The lidar parameters
    this->declare_parameter("lidar_type", "RS16");
    this->declare_parameter("input_type", "online");
    // The input parameters 
    this->declare_parameter("msop_port", 6699);           ///< Msop packet port number
    this->declare_parameter("difop_port", 7788);          ///< Difop packet port number
    this->declare_parameter("host_address", "0.0.0.0");   ///< Address of host
    this->declare_parameter("group_address", "0.0.0.0");  ///< Address of multicast group
    this->declare_parameter("pcap_path", "");             ///< Absolute path of pcap file
    this->declare_parameter("pcap_repeat", true);         ///< true: The pcap bag will repeat play
    this->declare_parameter("pcap_rate", 1.0f);           ///< Rate to read the pcap file
    this->declare_parameter("use_vlan", false);           ///< Vlan on-off
    this->declare_parameter("user_layer_bytes", 0);       ///< Bytes of user layer. thers is no user layer if it is 0
    this->declare_parameter("tail_layer_bytes", 0);       ///< Bytes of tail layer. thers is no tail layer if it is 0
    // The decoder parameters
    this->declare_parameter("wait_for_difop", true); ///< true: start sending point cloud until receive difop packet
    this->declare_parameter("min_distance", 0.0f);   ///< min/max distances of point cloud range. valid if min distance or max distance > 0
    this->declare_parameter("max_distance", 0.0f);
    this->declare_parameter("start_angle", 0.0f);    ///< Start angle of point cloud
    this->declare_parameter("end_angle", 360.0f);    ///< End angle of point cloud
    // this->declare_parameter("split_frame_mode", "split_by_angle");
                                 ///< 1: Split frames by split_angle;
                                 ///< 2: Split frames by fixed number of blocks;
                                 ///< 3: Split frames by custom number of blocks (num_blks_split)
    this->declare_parameter("split_angle", 0.0f);       ///< Split angle(degree) used to split frame, only be used when split_frame_mode=1
    this->declare_parameter("num_blks_split", 1);       ///< Number of packets in one frame, only be used when split_frame_mode=3
    this->declare_parameter("use_lidar_clock", false);  ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
    this->declare_parameter("dense_points", false);     ///< true: discard NAN points; false: reserve NAN points
    this->declare_parameter("ts_first_point", false);   ///< true: time-stamp point cloud with the first point; false: with the last point;
    // The point transform parameter
    this->declare_parameter("x", 0.0f);      ///< unit, m
    this->declare_parameter("y", 0.0f);      ///< unit, m
    this->declare_parameter("z", 0.0f);      ///< unit, m
    this->declare_parameter("roll", 0.0f);   ///< unit, radian
    this->declare_parameter("pitch", 0.0f);  ///< unit, radian
    this->declare_parameter("yaw", 0.0f);    ///< unit, radian
    // The output parameters
    this->declare_parameter("point_cloud_topic", "rslidar_points");
    this->declare_parameter("send_by_rows", false); /// will set to false if dense_points == true
    this->declare_parameter("frame_id", "rslidar"); ///< The frame id of LiDAR mesage
    
    // Get parameters
    std::string lidar_type = this->get_parameter("lidar_type").as_string();
    std::string input_type = this->get_parameter("input_type").as_string();

    int msop_port = this->get_parameter("msop_port").as_int();
    int difop_port = this->get_parameter("difop_port").as_int();
    std::string host_address = this->get_parameter("host_address").as_string();
    std::string group_address = this->get_parameter("group_address").as_string();
    std::string pcap_path = this->get_parameter("pcap_path").as_string();
    double pcap_rate = this->get_parameter("pcap_rate").as_double();
    bool pcap_repeat = this->get_parameter("pcap_repeat").as_bool();
    bool use_vlan = this->get_parameter("use_vlan").as_bool();
    int user_layer_bytes = this->get_parameter("user_layer_bytes").as_int();
    int tail_layer_bytes = this->get_parameter("tail_layer_bytes").as_int();

    bool wait_for_difop = this->get_parameter("wait_for_difop").as_bool();
    double min_distance = this->get_parameter("min_distance").as_double();
    double max_distance = this->get_parameter("max_distance").as_double();
    double start_angle = this->get_parameter("start_angle").as_double();
    double end_angle = this->get_parameter("end_angle").as_double();
    // std::string split_frame_mode = this->get_parameter("split_frame_mode").as_string();
    double split_angle = this->get_parameter("split_angle").as_double();
    int num_blks_split = this->get_parameter("num_blks_split").as_int();
    bool use_lidar_clock = this->get_parameter("use_lidar_clock").as_bool();
    bool dense_points = this->get_parameter("dense_points").as_bool(); 
    bool ts_first_point = this->get_parameter("ts_first_point").as_bool();

    double x = this->get_parameter("x").as_double();      
    double y = this->get_parameter("y").as_double();      
    double z = this->get_parameter("z").as_double();      
    double roll = this->get_parameter("roll").as_double();   
    double pitch = this->get_parameter("pitch").as_double();  
    double yaw = this->get_parameter("yaw").as_double();    
  
    std::string point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
    send_by_rows_ = this->get_parameter("send_by_rows").as_bool(); 
    frame_id_ = this->get_parameter("frame_id").as_string(); 

    // Set driver parameters based on the values from the launch file
    RSDriverParam param;
    param.lidar_type = strToLidarType(lidar_type);
    param.input_type = (input_type == "online") ? InputType::ONLINE_LIDAR : InputType::PCAP_FILE;

    param.input_param.msop_port = msop_port;
    param.input_param.difop_port = difop_port;
    param.input_param.host_address = host_address;
    param.input_param.pcap_path = pcap_path;
    param.input_param.pcap_rate = pcap_rate;
    param.input_param.pcap_repeat = pcap_repeat;
    param.input_param.use_vlan = use_vlan;
    param.input_param.user_layer_bytes = user_layer_bytes;
    param.input_param.tail_layer_bytes = tail_layer_bytes;

    param.decoder_param.wait_for_difop = wait_for_difop;
    param.decoder_param.min_distance = min_distance;
    param.decoder_param.max_distance = max_distance;
    param.decoder_param.start_angle = start_angle;
    param.decoder_param.end_angle = end_angle;
    //// split frame mode 
    param.decoder_param.split_angle = split_angle;
    param.decoder_param.num_blks_split = num_blks_split;
    param.decoder_param.use_lidar_clock = use_lidar_clock;
    param.decoder_param.dense_points = dense_points;
    param.decoder_param.ts_first_point = ts_first_point;

    param.decoder_param.transform_param.x = x;
    param.decoder_param.transform_param.y = y;
    param.decoder_param.transform_param.z = z;
    param.decoder_param.transform_param.roll = roll;
    param.decoder_param.transform_param.pitch = pitch;
    param.decoder_param.transform_param.yaw = yaw;

    param.frame_id = frame_id_;

    param.print();

    if (dense_points) 
    {
      send_by_rows_ = false;
    };


    // Initialize the driver with the parameters
    driver_ptr_.reset(new LidarDriver<LidarPointCloudMsg>());
    driver_ptr_->regPointCloudCallback(std::bind(&RSLidarDriver::getPointCloud, this), 
        std::bind(&RSLidarDriver::putPointCloud, this, std::placeholders::_1));
    driver_ptr_->regExceptionCallback(
        std::bind(&RSLidarDriver::exceptionCallback, this, std::placeholders::_1));

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic, rclcpp::SensorDataQoS());
    point_cloud_process_thread_ = std::thread(std::bind(&RSLidarDriver::processPointCloud, this));

    if (!driver_ptr_->init(param))
    {
      RCLCPP_ERROR(this->get_logger(), "Driver Initialize Error...");
      throw std::runtime_error("Driver Initialize Error");
    }

    // Start the driver
    driver_ptr_->start();
}

void RSLidarDriver::stop()
{
  driver_ptr_->stop();
  to_exit_process_ = true;
  point_cloud_process_thread_.join();
}

inline void RSLidarDriver::sendPointCloud(const LidarPointCloudMsg& msg)
{
  point_cloud_pub_->publish(toRosMsg(msg, frame_id_, send_by_rows_));
  // RCLCPP_INFO(this->get_logger(), "publish pointcloud with %s as frame_id", frame_id_.c_str());
}


void RSLidarDriver::putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg)
{
  point_cloud_queue_.push(msg);
}

void RSLidarDriver::processPointCloud()
{
  while (!to_exit_process_)
  {
    std::shared_ptr<LidarPointCloudMsg> msg = point_cloud_queue_.popWait(1000);
    if (msg.get() == NULL)
    {
      continue;
    }

    sendPointCloud(*msg);

    free_point_cloud_queue_.push(msg);
  }
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

void RSLidarDriver::exceptionCallback(const Error& code)
{
  code.toString();
  // RCLCPP_WARN(this->get_logger(), code.toString());
}

} // namespace robosense::lidar

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RSLidarDriver)