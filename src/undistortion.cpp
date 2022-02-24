
#include "undistortion.hpp"

void pointCallBack(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen3x4d Rt;

    // PointTypeCloud point_cloud(new pcl::PointCloud<PointType>());

    // pcl::PointCloud<PointType>::Ptr point_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

    // point_cloud_->clear();
    // pcl::fromROSMsg(*msg, *point_cloud);

    // scan_time_ = msg->header.stamp.toSec();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr l2I_points_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // for (auto &p:*point_cloud)
    // {
    //     pcl::PointXYZI pt;

    //     // Eigen::MatrixXd lidar_points ;
    //     // lidar_points << p.x , p.y, p.z, 1;

    //     Eigen4x1d lidar_points = Eigen4x1d::Identity();
    //     lidar_points << p.x , p.y, p.z, 1;

    //     Eigen::MatrixXd l2I = L2I_tm_ * lidar_points;

    //     pt.x = l2I(0);
    //     pt.y = l2I(1);
    //     pt.z = l2I(2);
    //     pt.intensity = p.intensity;

    //     l2I_points_cloud->push_back(pt);
    // }

    // sensor_msgs::PointCloud2 l2I_cloud_msg;
    // sensor_msgs::PointCloud2ConstPtr l2I_const_cloud_msg;
    // pcl::toROSMsg(*l2I_points_cloud, l2I_cloud_msg);
    // l2I_cloud_msg.header = msg->header;
    // l2I_cloud_pub.publish(l2I_cloud_msg);

    // sensor_msgs::PointCloud2ConstPtr l2I_const_cloud_msg;
    // l2I_const_cloud_msg->

    // push lidar msg to queue
    std::unique_lock<std::mutex> lock(mutexLidarQueue_);
    lidarMsgQueue_.push(msg);
}

void AccumulateIMUShiftAndRotation()
{
    float roll = imu_roll_[imu_ptr_last_];
    float pitch = imu_pitch_[imu_ptr_last_];
    float yaw = imu_yaw_[imu_ptr_last_];
    float accX = imu_acc_x_[imu_ptr_last_];
    float accY = imu_acc_y_[imu_ptr_last_];
    float accZ = imu_acc_z_[imu_ptr_last_];

    float x1 = cos(roll) * accX - sin(roll) * accY;
    float y1 = sin(roll) * accX + cos(roll) * accY;
    float z1 = accZ;

    float x2 = x1;
    float y2 = cos(pitch) * y1 - sin(pitch) * z1;
    float z2 = sin(pitch) * y1 + cos(pitch) * z1;

    accX = cos(yaw) * x2 + sin(yaw) * z2;
    accY = y2;
    accZ = -sin(yaw) * x2 + cos(yaw) * z2;

    int imu_ptr_back = (imu_ptr_last_ + imu_que_length_ - 1) % imu_que_length_;
    double timeDiff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
    if (timeDiff < scan_period_)
    {
        // 움직인 거리 = ut +1/2 + 가속도 * dt^2 -> 등가속도 운동 공식

        imu_shift_x_[imu_ptr_last_] =
            imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * timeDiff + accX * timeDiff * timeDiff / 2;
        imu_shift_y_[imu_ptr_last_] =
            imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * timeDiff + accY * timeDiff * timeDiff / 2;
        imu_shift_z_[imu_ptr_last_] =
            imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * timeDiff + accZ * timeDiff * timeDiff / 2;

        imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + accX * timeDiff;
        imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + accY * timeDiff;
        imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + accZ * timeDiff;

        imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + imu_angular_velo_x_[imu_ptr_back] * timeDiff;
        imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + imu_angular_velo_y_[imu_ptr_back] * timeDiff;
        imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + imu_angular_velo_z_[imu_ptr_back] * timeDiff;
    }
}

void distortionCloud()
{
    int cloudSize = point_cloud_->points.size();
    bool half_passed = false;

    if (cloudSize > 0)
    {
        bool halfPassed = false;
        float start_ori = -std::atan2(point_cloud_->points[0].y, point_cloud_->points[0].x);
        float end_ori = -std::atan2(point_cloud_->points[cloudSize - 1].y, point_cloud_->points[cloudSize - 1].x);
        if (end_ori - start_ori > 3 * M_PI)
        {
            end_ori -= 2 * M_PI;
        }
        else if (end_ori - start_ori < M_PI)
        {
            end_ori += 2 * M_PI;
        }
        float ori_diff = end_ori - start_ori;

        Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
        Eigen::Vector3f shift_from_start;
        Eigen::Matrix3f r_s_i, r_c;
        Eigen::Vector3f adjusted_p;
        float ori_h;

        for (int i = 0; i < cloudSize; ++i)
        {
            pcl::PointXYZI &p = point_cloud_->points[i];
            ori_h = -std::atan2(p.y, p.x);
            if (!half_passed)
            {
                if (ori_h < start_ori - M_PI * 0.5)
                {
                    ori_h += 2 * M_PI;
                }
                else if (ori_h > start_ori + M_PI * 1.5)
                {
                https: // github.com/rsasaki0109/lidar_undistortion
                    ori_h -= 2 * M_PI;
                }

                if (ori_h - start_ori > M_PI)
                {
                    half_passed = true;
                }
            }
            else
            {
                ori_h += 2 * M_PI;
                if (ori_h < end_ori - 1.5 * M_PI)
                {
                    ori_h += 2 * M_PI;
                }
                else if (ori_h > end_ori + 0.5 * M_PI)
                {
                    ori_h -= 2 * M_PI;
                }
            }

            float rel_time = (ori_h - start_ori) / ori_diff * scan_period_;

            if (imu_ptr_last_ > 0)
            {
                imu_ptr_front_ = imu_ptr_last_iter_;
                while (imu_ptr_front_ != imu_ptr_last_)
                {
                    if (scan_time_ + rel_time > imu_time_[imu_ptr_front_])
                    {
                        break;
                    }
                    imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_que_length_;
                }

                if (scan_time_ + rel_time > imu_time_[imu_ptr_front_])
                {
                    rpy_cur(0) = imu_roll_[imu_ptr_front_];
                    rpy_cur(1) = imu_pitch_[imu_ptr_front_];
                    rpy_cur(2) = imu_yaw_[imu_ptr_front_];
                    shift_cur(0) = imu_shift_x_[imu_ptr_front_];
                    shift_cur(1) = imu_shift_y_[imu_ptr_front_];
                    shift_cur(2) = imu_shift_z_[imu_ptr_front_];
                    velo_cur(0) = imu_velo_x_[imu_ptr_front_];
                    velo_cur(1) = imu_velo_y_[imu_ptr_front_];
                    velo_cur(2) = imu_velo_z_[imu_ptr_front_];
                }
                else
                {
                    int imu_ptr_back = (imu_ptr_front_ - 1 + imu_que_length_) % imu_que_length_;
                    float ratio_front = (scan_time_ + rel_time - imu_time_[imu_ptr_back]) /
                                        (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
                    float ratio_back = 1.0 - ratio_front;
                    rpy_cur(0) = imu_roll_[imu_ptr_front_] * ratio_front + imu_roll_[imu_ptr_back] * ratio_back;
                    rpy_cur(1) = imu_pitch_[imu_ptr_front_] * ratio_front + imu_pitch_[imu_ptr_back] * ratio_back;
                    rpy_cur(2) = imu_yaw_[imu_ptr_front_] * ratio_front + imu_yaw_[imu_ptr_back] * ratio_back;
                    shift_cur(0) = imu_shift_x_[imu_ptr_front_] * ratio_front + imu_shift_x_[imu_ptr_back] * ratio_back;
                    shift_cur(1) = imu_shift_y_[imu_ptr_front_] * ratio_front + imu_shift_y_[imu_ptr_back] * ratio_back;
                    shift_cur(2) = imu_shift_z_[imu_ptr_front_] * ratio_front + imu_shift_z_[imu_ptr_back] * ratio_back;
                    velo_cur(0) = imu_velo_x_[imu_ptr_front_] * ratio_front + imu_velo_x_[imu_ptr_back] * ratio_back;
                    velo_cur(1) = imu_velo_y_[imu_ptr_front_] * ratio_front + imu_velo_y_[imu_ptr_back] * ratio_back;
                    velo_cur(2) = imu_velo_z_[imu_ptr_front_] * ratio_front + imu_velo_z_[imu_ptr_back] * ratio_back;
                }

                r_c = (Eigen::AngleAxisf(rpy_cur(2), Eigen::Vector3f::UnitZ()) *
                       Eigen::AngleAxisf(rpy_cur(1), Eigen::Vector3f::UnitY()) *
                       Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX()))
                          .toRotationMatrix();

                if (i == 0)
                {
                    rpy_start = rpy_cur;
                    shift_start = shift_cur;
                    velo_start = velo_cur;
                    r_s_i = r_c.inverse();
                }
                else
                {
                    shift_from_start = shift_cur - shift_start - velo_start * rel_time;
                    adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
                    p.x = adjusted_p.x();
                    p.y = adjusted_p.y();
                    p.z = adjusted_p.z();
                }
            }
            point_cloud_->points[i] = p;
            imu_ptr_last_iter_ = imu_ptr_front_;
        }
        sensor_msgs::PointCloud2 l2I_cloud_msg;
        sensor_msgs::PointCloud2ConstPtr l2I_const_cloud_msg;
        pcl::toROSMsg(*point_cloud_, l2I_cloud_msg);
        l2I_cloud_msg.header.frame_id = "map";
        l2I_cloud_msg.header.stamp = ros::Time::now();
        l2I_cloud_pub.publish(l2I_cloud_msg);
    }
}
void imuCallBack(const sensor_msgs::ImuConstPtr &imuIn)
{
    std::unique_lock<std::mutex> lock(mutexIMUQueue_);
    imuMsgQueue_.push(imuIn);

    double roll, pitch, yaw;
    tf::Quaternion orientation;

    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // std::cout << "orientation : " << orientation << "\n";

    // 직교좌표계 -> 구면좌표계

    float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
    float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
    float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

    imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_que_length_;
    imu_time_[imu_ptr_last_] = imuIn->header.stamp.toSec();

    imu_roll_[imu_ptr_last_] = roll;
    imu_pitch_[imu_ptr_last_] = pitch;
    imu_yaw_[imu_ptr_last_] = yaw;

    imu_acc_x_[imu_ptr_last_] = accX;
    imu_acc_y_[imu_ptr_last_] = accY;
    imu_acc_z_[imu_ptr_last_] = accZ;
    imu_angular_velo_x_[imu_ptr_last_] = imuIn->angular_velocity.x;
    imu_angular_velo_y_[imu_ptr_last_] = imuIn->angular_velocity.y;
    imu_angular_velo_z_[imu_ptr_last_] = imuIn->angular_velocity.z;

    // Eigen::Quaternionf quat;
    // quat.x = imuIn->orientation.x;
    // quat.y = imuIn->orientation.y;
    // quat.z = imuIn->orientation.z;
    // quat.w = imuIn->orientation.w;

    // quat.x

    // double roll, pitch, yaw;
    // tf::Quaternion orientation;
    // tf::quaternionMsgToTF(imuIn->orientation, orientation);
    // tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // float roll, pitch, yaw;
    // Eigen::Affine3f affine(quat);
    // pcl::getEulerAngles(affine, roll, pitch, yaw);

    // imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_que_length_;

    // if ((imu_ptr_last_ + 1) % imu_que_length_ == imu_ptr_front_) {
    //     imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_que_length_;
    // }

    // imu_time_[imu_ptr_last_] = imu_time;
    // imu_roll_[imu_ptr_last_] = roll;
    // imu_pitch_[imu_ptr_last_] = pitch;
    // imu_yaw_[imu_ptr_last_] = yaw;
    // imu_acc_x_[imu_ptr_last_] = acc.x();
    // imu_acc_y_[imu_ptr_last_] = acc.y();
    // imu_acc_z_[imu_ptr_last_] = acc.z();
    // imu_angular_velo_x_[imu_ptr_last_] = angular_velo.x();
    // imu_angular_velo_y_[imu_ptr_last_] = angular_velo.y();
    // imu_angular_velo_z_[imu_ptr_last_] = angular_velo.z();

    // Eigen::Matrix3f rot = quat.toRotationMatrix();
    // acc = rot * acc;
    // angular_velo = rot * angular_velo;

    // int imu_ptr_back = (imu_ptr_last_ - 1 + imu_que_length_) % imu_que_length_;
    // double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
    // if (time_diff < scan_period_) {
    //     imu_shift_x_[imu_ptr_last_] =
    //     imu_shift_x_[imu_ptr_back] +imu_velo_x_[imu_ptr_back] * time_diff + acc(0) * time_diff * time_diff * 0.5;
    //     imu_shift_y_[imu_ptr_last_] =
    //     imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff + acc(1) * time_diff * time_diff * 0.5;
    //     imu_shift_z_[imu_ptr_last_] =
    //     imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff + acc(2) * time_diff * time_diff * 0.5;

    //     imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
    //     imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
    //     imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;

    //     imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + angular_velo(0) * time_diff;
    //     imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + angular_velo(1) * time_diff;
    //     imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + angular_velo(2) * time_diff;
    // }

    // AccumulateIMUShiftAndRotation();
}

bool imuTimeSync(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimuMsg, sensor_msgs::ImuConstPtr &lastImuMsg)
{

    /** \brief 특정시간(LiDAR와의 sync)대의 IMU 데이터를 가져오는 함수
     * \param[in] startTime: t 시간대의 lidar 시간
     * \param[in] endTime: t+1 시간대의 lidar 시간
     * \param[in] vimuMsg: IMU data vector
     */

    std::unique_lock<std::mutex> lock(mutexIMUQueue_);
    double current_time = 0;

    vimuMsg.clear();

    bool lastImuFlg = false;
    
    if(lastImuMsg != 0)
    {
        vimuMsg.push_back(lastImuMsg);
    }

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
        if (time <= endTime && time >= startTime)
        {
            // std::cout << std::fixed<<">>time : " << time << "\n";
            // std::cout << std::fixed<<">>endTime : " << endTime << "\n";
            // std::cout << std::fixed<<">>startTime : " << startTime << "\n";
            // std::cout << " ================" << "\n";
            // exit(1);
            
             
            
            vimuMsg.push_back(tmpimumsg);
            // qimuMsg_.push(tmpimumsg);
            
            current_time = time;
            imuMsgQueue_.pop();

            // std::cout << vimuMsg.size() << "\n";
            if (time == endTime)
                break;
        }
        

        // else
        // {
        //     std::cout << "else check "
        //               << "\n";
        //               exit(1);
            if (time <= startTime)
            {
                imuMsgQueue_.pop();
            }
        //     else
        //     {
        //         double dt_1 = endTime - current_time;
        //         double dt_2 = time - endTime;

        //         ROS_ASSERT(dt_1 >= 0);
        //         ROS_ASSERT(dt_2 >= 0);
        //         ROS_ASSERT(dt_1 + dt_2 > 0);
        //         double w1 = dt_2 / (dt_1 + dt_2);
        //         double w2 = dt_1 / (dt_1 + dt_2);
        //         sensor_msgs::ImuPtr theLastIMU(new sensor_msgs::Imu);
        //         theLastIMU->linear_acceleration.x = w1 * vimuMsg.back()->linear_acceleration.x + w2 * tmpimumsg->linear_acceleration.x;
        //         theLastIMU->linear_acceleration.y = w1 * vimuMsg.back()->linear_acceleration.y + w2 * tmpimumsg->linear_acceleration.y;
        //         theLastIMU->linear_acceleration.z = w1 * vimuMsg.back()->linear_acceleration.z + w2 * tmpimumsg->linear_acceleration.z;
        //         theLastIMU->angular_velocity.x = w1 * vimuMsg.back()->angular_velocity.x + w2 * tmpimumsg->angular_velocity.x;
        //         theLastIMU->angular_velocity.y = w1 * vimuMsg.back()->angular_velocity.y + w2 * tmpimumsg->angular_velocity.y;
        //         theLastIMU->angular_velocity.z = w1 * vimuMsg.back()->angular_velocity.z + w2 * tmpimumsg->angular_velocity.z;
        //         theLastIMU->header.stamp.fromSec(endTime);
        //         vimuMsg.emplace_back(theLastIMU);
        //         break;
        //     }
        // }
    }
    return !vimuMsg.empty();
}

void process()
{

    double time_last_lidar = -1;
    double time_curr_lidar = -1;

    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;
    // std::queue<sensor_msgs::ImuConstPtr> qimuMsg;

    sensor_msgs::ImuConstPtr lastImuMsg;

    while (ros::ok())
    {
        currCloud_.reset(new pcl::PointCloud<PointType>);
        // lastCloud_.reset(new pcl::PointCloud<PointType>);

        std::unique_lock<std::mutex> lock_lidar(mutexLidarQueue_);

        if (!lidarMsgQueue_.empty())
        {
            // get new lidar msg
            time_curr_lidar = lidarMsgQueue_.front()->header.stamp.toSec();
            pcl::fromROSMsg(*lidarMsgQueue_.front(), *currCloud_);

            lidarMsgQueue_.pop();
            newfullCloud_ = true;
            scanIdx_++;
            // std::cout << scanIdx_ << "\n";

        }
        else
        {
            newfullCloud_ = false;
        }
        
        lock_lidar.unlock();

        if (newfullCloud_)
        {
            if (scanIdx_ >= 3)
            {
                // std::cout << scanIdx_ << "\n";
                if (time_last_lidar > 0)
                {
                    // vimuMsg.clear();
                    int countFail = 0;
                    // std::cout << "in" << "\n";
                    while (!imuTimeSync(time_last_lidar, time_curr_lidar, vimuMsg, lastImuMsg))
                    {

                        countFail++;
                        if (countFail > 100)
                        {
                            break;
                        }
                        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                }

                if (!vimuMsg.empty())
                {
                    

                    std::cout << " cur size : " << currCloud_->size() << "\n";
                    std::cout << "lidar size : " << lastCloud_->points.size() << "\n";
                    
                    std::cout << std::fixed << "lidar [0] :" << lastCloud_->points[0].timestamp << "\n";

                    std::cout << "vimuMsg.size() : " << vimuMsg.size() << "\n";

                    std::cout << "vimmsg [0] : " << vimuMsg[0]->header.stamp.toSec() << "\n";

                    std::cout << "vimmsg [1] : " << vimuMsg[1]->header.stamp.toSec() << "\n";
                    std::cout << "vimmsg [E] : " << vimuMsg[vimuMsg.size() - 1]->header.stamp.toSec() << "\n";
                    

                    std::cout << "lidar [E] :" << lastCloud_->points[lastCloud_->size() - 1].timestamp << "\n";
                    std::cout << "\n";

                    // std::cout << lastImuMsg->header.stamp.toSec() << "\n";
                    std::cout << "================"
                              << "\n";

                    lastImuMsg = vimuMsg[vimuMsg.size()-1];

                }

                time_last_lidar = time_curr_lidar;

                // std::cout << "1" <<"\n";
                // std::cout << lastImuMsg->header.stamp.toSec() << "\n";
                // std::cout << "vimusize : " << vimuMsg[vimuMsg.size()-1] << "\n";
                

                
                lastCloud_->clear();
                
                for (auto &p : currCloud_->points)
                {
                    PointType points;
                    points.x = p.x;
                    points.y = p.y;
                    points.z = p.z;
                    points.intensity = p.intensity;
                    points.timestamp = p.timestamp;
                    lastCloud_->push_back(points);
                }
            }
            // lastCloud_->insert(currCloud_);
        }
    }
}

int main(int argc, char **argv)
{

    std::vector<double> vecTlb;
    int imu_mode;
    std::string point_cloud_topic;
    std::string imu_topic;
    std::vector<double> vecL2I;

    ros::init(argc, argv, "lidar undistortion");
    ros::NodeHandle nodeHandler("~");
    ros::param::get("~IMU_Mode", imu_mode);
    ros::param::get("~point_cloud_topic", point_cloud_topic);
    ros::param::get("~imu_topic", imu_topic);
    ros::param::get("~Extrinsic_L2I", vecL2I);

    Eigen::AngleAxisd L2I_x_rot(DEG2RAD(vecL2I[0]), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd L2I_y_rot(DEG2RAD(vecL2I[1]), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd L2I_z_rot(DEG2RAD(vecL2I[2]), Eigen::Vector3d::UnitZ());

    Eigen::Translation3d L2I_tl(vecL2I[3], vecL2I[4], vecL2I[5]);

    currCloud_.reset(new pcl::PointCloud<PointType>);
    lastCloud_.reset(new pcl::PointCloud<PointType>);

    point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    currCloud_.reset(new pcl::PointCloud<PointType>);

    // lastCloud_ = PointTypeCloud(new pcl::PointCloud<PointType>);

    // pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

    L2I_tm_ = (L2I_tl * L2I_z_rot * L2I_y_rot * L2I_x_rot).matrix().matrix().block(0, 0, 3, 4);
    // ros::Subscriber subFullCloud = nodeHandler.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic, 10, pointCallBack);

    ros::Subscriber sub_imu = nodeHandler.subscribe(imu_topic, 2000, imuCallBack);
    ros::Subscriber subFullCloud = nodeHandler.subscribe(point_cloud_topic, 10, pointCallBack);
    // ros::Subscriber sub_imu = nodeHandler.subscribe(imu_topic, 2000, imuCallBack)

    // ros::TransportHints() -> net work 통신 방법 지정해줌
    // unrealiable : 신뢰할 수 없는 전송을 지정. UDP라는데 정확한 설명 X

    l2I_cloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("l2I_cloud_topic", 1, true);

    // ros::Rate rate(200);
    // while (ros::ok())
    // {

    //     ros::spinOnce();

    //     distortionCloud();

    //     rate.sleep();
    // }

    std::thread thread_process{process};

    ros::spin();

    return 1;
}