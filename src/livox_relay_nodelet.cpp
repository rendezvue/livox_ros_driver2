#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>

namespace livox_ros
{

class LivoxRelayNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit()
    {
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& pnh = getPrivateNodeHandle();

        pub_ = nh.advertise<sensor_msgs::PointCloud2>("lidar_nodelet", 1);
        sub_ = nh.subscribe("/livox/lidar", 1, &LivoxRelayNodelet::callback, this);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 필요시 가공 후 재발행
        pub_.publish(msg);
    }

  private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

} // namespace livox_ros

PLUGINLIB_EXPORT_CLASS(livox_ros::LivoxRelayNodelet, nodelet::Nodelet)
