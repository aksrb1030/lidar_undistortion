#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <sensor_msgs/Imu.h>
#include <queue>
#include <iterator>
#include <future>
#include <chrono>

int num = 0;

bool newfullCloud_ = false;

using Eigen3x4d = Eigen::Matrix<double, 3, 4>;
using Eigen3x1d = Eigen::Matrix<double, 3, 1>;
using Eigen4x1d = Eigen::Matrix<double, 4, 1>;




pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes_;

std::mutex mutexLidarQueue_;
std::queue<sensor_msgs::PointCloud2ConstPtr> lidarMsgQueue_;
std::mutex mutexIMUQueue_;
std::queue<sensor_msgs::ImuConstPtr> imuMsgQueue_;

Eigen::Matrix4d L2I_tm_;

ros::Publisher l2I_cloud_pub;

void pointCallBack(const sensor_msgs::PointCloud2ConstPtr &msg);
void imuCallBack(const sensor_msgs::ImuConstPtr &msg);
void process();
bool imuTimeSync(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimuMsg);