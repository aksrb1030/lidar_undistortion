
#include "undistortion.hpp"

void pointCallBack(const sensor_msgs::PointCloud2ConstPtr &msg)
{

    // push lidar msg to queue
    std::unique_lock<std::mutex> lock(mutexLidarQueue_);
    lidarMsgQueue_.push(msg);
}

void imuCallBack(const sensor_msgs::ImuConstPtr &msg)
{

    // push lidar msg to queue
    std::unique_lock<std::mutex> lock(mutexIMUQueue_);
    imuMsgQueue_.push(msg);
}

void imuTimeCheck(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimuMsg)
{
    

    std::unique_lock<std::mutex> lock(mutexIMUQueue_);
    double current_time = 0;
    vimuMsg.clear();
    while (true)
    {
        if (imuMsgQueue_.empty())
            break;
        if (imuMsgQueue_.back()->header.stamp.toSec() < endTime ||
            imuMsgQueue_.front()->header.stamp.toSec() >= endTime)
            break;
        sensor_msgs::ImuConstPtr &tmpimumsg = imuMsgQueue_.front();
        double time = tmpimumsg->header.stamp.toSec();
        if (time <= endTime && time > startTime)
        {

            std::cout<< std::fixed << "startTime : " << startTime << "\n";
            std::cout<< std::fixed << "endTime : " << endTime << "\n";
            std::cout<< std::fixed << "time : " << time << "\n";
            std::cout << "==================================================" << "\n";

            vimuMsg.push_back(tmpimumsg);
            current_time = time;
            imuMsgQueue_.pop();
            if (time == endTime)
                break;
        }
        else
        {
            if (time <= startTime)
            {
                imuMsgQueue_.pop();
            }
            else
            {
                double dt_1 = endTime - current_time;
                double dt_2 = time - endTime;
                ROS_ASSERT(dt_1 >= 0);
                ROS_ASSERT(dt_2 >= 0);
                ROS_ASSERT(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                sensor_msgs::ImuPtr theLastIMU(new sensor_msgs::Imu);
                theLastIMU->linear_acceleration.x = w1 * vimuMsg.back()->linear_acceleration.x + w2 * tmpimumsg->linear_acceleration.x;
                theLastIMU->linear_acceleration.y = w1 * vimuMsg.back()->linear_acceleration.y + w2 * tmpimumsg->linear_acceleration.y;
                theLastIMU->linear_acceleration.z = w1 * vimuMsg.back()->linear_acceleration.z + w2 * tmpimumsg->linear_acceleration.z;
                theLastIMU->angular_velocity.x = w1 * vimuMsg.back()->angular_velocity.x + w2 * tmpimumsg->angular_velocity.x;
                theLastIMU->angular_velocity.y = w1 * vimuMsg.back()->angular_velocity.y + w2 * tmpimumsg->angular_velocity.y;
                theLastIMU->angular_velocity.z = w1 * vimuMsg.back()->angular_velocity.z + w2 * tmpimumsg->angular_velocity.z;
                theLastIMU->header.stamp.fromSec(endTime);
                vimuMsg.emplace_back(theLastIMU);
                break;
            }
        }
    }
}

void process()
{
    double time_last_lidar = -1;
    double time_curr_lidar = -1;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

    while (ros::ok())
    {
        laserCloudFullRes_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        std::unique_lock<std::mutex> lock_lidar(mutexLidarQueue_);

        if (!lidarMsgQueue_.empty())
        {
            // get new lidar msg
            time_curr_lidar = lidarMsgQueue_.front()->header.stamp.toSec();
            // std::cout << " lidarMsgQueue_.front()->header.stamp.toSec() : " << lidarMsgQueue_.front()->header.stamp.toSec() << "\n";

            pcl::fromROSMsg(*lidarMsgQueue_.front(), *laserCloudFullRes_);
            lidarMsgQueue_.pop();
            newfullCloud_ = true;
        }
        lock_lidar.unlock();

        if (newfullCloud_)
        {
            nav_msgs::Odometry debugInfo;
            debugInfo.pose.pose.position.x = 0;
            debugInfo.pose.pose.position.y = 0;
            debugInfo.pose.pose.position.z = 0;

            if (time_last_lidar > 0)
            {
                // get IMU msg int the Specified time interval
                vimuMsg.clear();
                imuTimeCheck(time_last_lidar, time_curr_lidar, vimuMsg);

                /* code */
            }

            time_last_lidar = time_curr_lidar;


        }
    }
}

int main(int argc, char **argv)
{

    std::vector<double> vecTlb;
    int imu_mode;
    std::string point_cloud_topic;
    std::string imu_topic;

    ros::init(argc, argv, "PoseEstimation");
    ros::NodeHandle nodeHandler("~");
    ros::param::get("~IMU_Mode", imu_mode);
    ros::param::get("~point_cloud_topic", point_cloud_topic);
    ros::param::get("~imu_topic", imu_topic);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    // std::cout << R << "\n";
    // std::cout << t << "\n";
    ros::Subscriber subFullCloud = nodeHandler.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic, 10, pointCallBack);
    // ros::Subscriber sub_imu = nodeHandler.subscribe(imu_topic, 2000, imuCallBack)

    // ros::TransportHints() -> net work 통신 방법 지정해줌
    // unrealiable : 신뢰할 수 없는 전송을 지정. UDP라는데 정확한 설명 X
    ros::Subscriber sub_imu = nodeHandler.subscribe(imu_topic, 2000, imuCallBack, ros::TransportHints().unreliable());

    laserCloudFullRes_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    std::thread thread_process{process};

    ros::spin();

    return 1;
}