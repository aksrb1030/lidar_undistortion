
#include "undistortion.hpp"

void pointCallBack(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen3x4d Rt;

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *point_cloud);

    // std::cout << Rt <<"\n";
    // std::cout << Rt.size() <<"\n";
    // std::cout <<"Rt.cols() : " <<Rt.cols() <<"\n";
    // std::cout <<"Rt.rows() : " <<Rt.rows() <<"\n";

    Eigen3x1d l2c = Eigen3x1d::Identity();
    // std::cout << " l2c : " << l2c.size() << "\n";
    // std::cout <<"l2c.cols() : " <<l2c.cols() <<"\n";
    // std::cout <<"l2c.rows() : " <<l2c.rows() <<"\n";
    // std::cout << "==============================" << "\n";
    // Rt.

    // std::cout << "point_cloud->height : "<<point_cloud->height <<"\n";
    // std::cout << "point_cloud->width : "<<point_cloud->width <<"\n";

    pcl::PointCloud<pcl::PointXYZI>::Ptr l2I_points_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (auto &p:*point_cloud)
    {
        pcl::PointXYZI pt;

        // Eigen::MatrixXd lidar_points ;
        // lidar_points << p.x , p.y, p.z, 1;

        Eigen4x1d lidar_points = Eigen4x1d::Identity();
        lidar_points << p.x , p.y, p.z, 1;

        Eigen::MatrixXd l2I = L2I_tm_ * lidar_points;

        pt.x = l2I(0);
        pt.y = l2I(1);
        pt.z = l2I(2);
        pt.intensity = p.intensity;

        l2I_points_cloud->push_back(pt);
    }
    
    sensor_msgs::PointCloud2 l2I_cloud_msg;
    pcl::toROSMsg(*l2I_points_cloud, l2I_cloud_msg);
    l2I_cloud_msg.header = msg->header;
    l2I_cloud_pub.publish(l2I_cloud_msg);
    


    
    // push lidar msg to queue
    // std::unique_lock<std::mutex> lock(mutexLidarQueue_);


    // lidarMsgQueue_.push(msg);
}

void imuCallBack(const sensor_msgs::ImuConstPtr &msg)
{

    // push lidar msg to queue
    std::unique_lock<std::mutex> lock(mutexIMUQueue_);
    imuMsgQueue_.push(msg);
}

bool imuTimeSync(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimuMsg)
{

    /** \brief 특정시간(LiDAR와의 sync)대의 IMU 데이터를 가져오는 함수
     * \param[in] startTime: t 시간대의 lidar 시간
     * \param[in] endTime: t+1 시간대의 lidar 시간
     * \param[in] vimuMsg: IMU data vector
     */

    std::unique_lock<std::mutex> lock(mutexIMUQueue_);
    double current_time = 0;
    vimuMsg.clear();
    while (true)
    {
        if (imuMsgQueue_.empty())
        {
            break;
        }

        // t+1이 IMU back time 보다 크거나 t+1값이 IMU front time 보다 작으면
        if (imuMsgQueue_.back()->header.stamp.toSec() < endTime ||
            imuMsgQueue_.front()->header.stamp.toSec() >= endTime)
        {
            break;
        }

        sensor_msgs::ImuConstPtr &tmpimumsg = imuMsgQueue_.front();
        double time = tmpimumsg->header.stamp.toSec();
        if (time <= endTime && time > startTime)
        {
            vimuMsg.push_back(tmpimumsg);
            current_time = time;
            imuMsgQueue_.pop();

            // std::cout << vimuMsg.size() << "\n";
            if (time == endTime)
                break;
        }
        else
        {
            std::cout << "else check " << "\n";
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
    return !vimuMsg.empty();
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
                imuTimeSync(time_last_lidar, time_curr_lidar, vimuMsg);

                // std::cout <<vimuMsg.size() << "\n";

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
    std::vector<double>  vecL2I;

    ros::init(argc, argv, "PoseEstimation");
    ros::NodeHandle nodeHandler("~");
    ros::param::get("~IMU_Mode", imu_mode);
    ros::param::get("~point_cloud_topic", point_cloud_topic);
    ros::param::get("~imu_topic", imu_topic);
    ros::param::get("~Extrinsic_L2I", vecL2I);



    Eigen::AngleAxisd L2I_x_rot(DEG2RAD(vecL2I[0]), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd L2I_y_rot(DEG2RAD(vecL2I[1]), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd L2I_z_rot(DEG2RAD(vecL2I[2]), Eigen::Vector3d::UnitZ());
    
    Eigen::Translation3d L2I_tl(vecL2I[3], vecL2I[4], vecL2I[5]);

    L2I_tm_ = (L2I_tl * L2I_z_rot * L2I_y_rot * L2I_x_rot).matrix().matrix().block(0, 0, 3, 4);

    ros::Subscriber subFullCloud = nodeHandler.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic, 10, pointCallBack);
    // ros::Subscriber sub_imu = nodeHandler.subscribe(imu_topic, 2000, imuCallBack)

    // ros::TransportHints() -> net work 통신 방법 지정해줌
    // unrealiable : 신뢰할 수 없는 전송을 지정. UDP라는데 정확한 설명 X
    ros::Subscriber sub_imu = nodeHandler.subscribe(imu_topic, 2000, imuCallBack, ros::TransportHints().unreliable());

    laserCloudFullRes_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    l2I_cloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("l2I_cloud_topic", 1, true);

    // std::thread thread_process{process};

    ros::spin();

    return 1;
}