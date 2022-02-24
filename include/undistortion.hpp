#include <iostream>
#include <queue>
#include <iterator>
#include <future>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

struct PointXYZIT {
  PCL_ADD_POINT4D   
  float intensity;
  double timestamp;
  uint16_t ring;                   
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;                   

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))

typedef PointXYZIT PointType;
typedef pcl::PointCloud<PointType>::Ptr PointTypeCloud;


int num = 0;

int scanIdx_ = 0;

bool newfullCloud_ = false;

using Eigen3x4d = Eigen::Matrix<double, 3, 4>;
using Eigen3x1d = Eigen::Matrix<double, 3, 1>;
using Eigen4x1d = Eigen::Matrix<double, 4, 1>;



double lidarTimeOffset_ = 0.3;

pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_;
PointTypeCloud laserCloudFullRes_;

PointTypeCloud currCloud_;
PointTypeCloud lastCloud_;

sensor_msgs::ImuConstPtr lastImuMsg_;

std::mutex mutexLidarQueue_;
std::queue<sensor_msgs::PointCloud2ConstPtr> lidarMsgQueue_;

std::mutex mutexIMUQueue_;
std::queue<sensor_msgs::ImuConstPtr> imuMsgQueue_;

Eigen::MatrixXd L2I_tm_;

ros::Publisher l2I_cloud_pub;

double scan_time_;


double lidar_curr_t_;
double lidar_last_t_;

double scan_period_{0.1};
static const int imu_que_length_{200};
int imu_ptr_front_{0}, imu_ptr_last_{-1}, imu_ptr_last_iter_{0};

std::array<double, imu_que_length_> imu_time_;
std::array<float, imu_que_length_> imu_roll_;
std::array<float, imu_que_length_> imu_pitch_;
std::array<float, imu_que_length_> imu_yaw_;

std::array<float, imu_que_length_> imu_acc_x_;
std::array<float, imu_que_length_> imu_acc_y_;
std::array<float, imu_que_length_> imu_acc_z_;
std::array<float, imu_que_length_> imu_velo_x_;
std::array<float, imu_que_length_> imu_velo_y_;
std::array<float, imu_que_length_> imu_velo_z_;
std::array<float, imu_que_length_> imu_shift_x_;
std::array<float, imu_que_length_> imu_shift_y_;
std::array<float, imu_que_length_> imu_shift_z_;

std::array<float, imu_que_length_> imu_angular_velo_x_;
std::array<float, imu_que_length_> imu_angular_velo_y_;
std::array<float, imu_que_length_> imu_angular_velo_z_;
std::array<float, imu_que_length_> imu_angular_rot_x_;
std::array<float, imu_que_length_> imu_angular_rot_y_;
std::array<float, imu_que_length_> imu_angular_rot_z_;


void pointCallBack(const sensor_msgs::PointCloud2ConstPtr &msg);
void imuCallBack(const sensor_msgs::ImuConstPtr &msg);
void AccumulateIMUShiftAndRotation();
void process();
void distortionCloud();
bool imuTimeSync(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimuMsg, sensor_msgs::ImuConstPtr &lastImuMsg);