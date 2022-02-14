/**
* Created by wangzb on 2022/1/16.
* 代码说明：
* 1，用于北京理工大学机电学院无人车自动驾驶实验平台
* 2，将csv数据转换为rosbag，数据包含OXTS, HDL, ZED等硬件数据
* 3，数据读取格式与实验平台的驱动代码相对应
**/

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <GeographicLib/LocalCartesian.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "csv.h"

using namespace std;
static GeographicLib::LocalCartesian geo_converter;


const double WGS84_A = 6378137.0;		// major axis
const double WGS84_B = 6356752.31424518;	// minor axis
const double WGS84_F = 0.0033528107;		// ellipsoid flattening
const double WGS84_E = 0.0818191908;		// first eccentricity
const double WGS84_EP = 0.0820944379;		// second eccentricity

// UTM Parameters
const double UTM_K0 = 0.9996;			// scale factor
const double UTM_FE = 500000.0;		// false easting
const double UTM_FN_N = 0.0;			// false northing on north hemisphere
const double UTM_FN_S = 10000000.0;		// false northing on south hemisphere
const double UTM_E2 = (WGS84_E*WGS84_E);	// e^2
const double UTM_E4 = (UTM_E2*UTM_E2);		// e^4
const double UTM_E6 = (UTM_E4*UTM_E2);		// e^6
const double UTM_EP2 = (UTM_E2/(1-UTM_E2));	// e'^2

const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;

char UTMLetterDesignator(double Lat)
{
    char LetterDesignator;

    if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
    else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
    else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
    else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
    else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
    else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
    else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
    else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
    else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
    else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
    else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
    else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
    else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
    else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
    else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
    else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
    else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
    else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
    else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
    else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
        // 'Z' is an error flag, the Latitude is outside the UTM limits
    else LetterDesignator = 'Z';
    return LetterDesignator;
}

//经纬度转为UTM坐标
void LLtoUTM(const double Lat, const double Long,
             double &UTMNorthing,
             double &UTMEasting,
             std::string &UTMZone)
{
    double a = WGS84_A;
    double eccSquared = UTM_E2;
    double k0 = UTM_K0;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;

    //Make sure the longitude is between -180.00 .. 179.9
    double LongTemp = (Long+180)-int((Long+180)/360)*360-180;

    double LatRad = Lat*RADIANS_PER_DEGREE;
    double LongRad = LongTemp*RADIANS_PER_DEGREE;
    double LongOriginRad;
    int    ZoneNumber;

    ZoneNumber = int((LongTemp + 180)/6) + 1;

    if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
        ZoneNumber = 32;

    // Special zones for Svalbard
    if( Lat >= 72.0 && Lat < 84.0 )
    {
        if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
        else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
        else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
        else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
    }
    // +3 puts origin in middle of zone
    LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
    LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

    //compute the UTM Zone from the latitude and longitude
    char zone_buf[] = {0,0,0,0};
    snprintf(zone_buf, 4, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
    UTMZone = zone_buf;
    eccPrimeSquared = (eccSquared)/(1-eccSquared);

    N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
    T = tan(LatRad)*tan(LatRad);
    C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
    A = cos(LatRad)*(LongRad-LongOriginRad);

    M = a*((1	- eccSquared/4		- 3*eccSquared*eccSquared/64	- 5*eccSquared*eccSquared*eccSquared/256)*LatRad
           - (3*eccSquared/8	+ 3*eccSquared*eccSquared/32	+ 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
           + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
           - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

    UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
                                + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
                          + 500000.0);

    UTMNorthing = (double)(k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                                                + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
    if(Lat < 0)
        UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
}

//for test
void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data(new
        pcl::PointCloud<pcl::PointXYZI>);
    long count = cloud_msg_ptr->data.size();
    pcl::fromROSMsg(*cloud_msg_ptr,*cloud_data);

    long pcl_count = cloud_data->points.size();
    std::cout<<"lidar size: "<<count<<std::endl;
    std::cout<<"pcl lidar size: "<<pcl_count<<std::endl;
}

//rosbag写入OXTS数据
bool OXTS_data2bag(rosbag::Bag &bag, string path, int fileNum = 1)
{
    string file_path = path + "OXTS/";
    for (int i = 0; i < fileNum; ++i)
    {
        string file_path = path + "OXTS/oxts_"+to_string(i+1) + ".csv";
        io::CSVReader<45> in(file_path);
        in.read_header(io::ignore_extra_column,"NavStatus", "timestamp", "localTimestamp",
                       "Lat", "Long", "Alt",
                       "East_Vel", "North_Vel", "Up_Vel",
                       "gps_heading", "gps_pitch", "gps_roll",
                       "gps_pos_cov_x", "gps_pos_cov_y", "gps_pos_cov_z",
                       "gps_vel_cov_x", "gps_vel_cov_y", "gps_vel_cov_z",
                       "liear_acc_x", "liear_acc_y", "liear_acc_z",
                       "angular_vel_x","angular_vel_y","angular_vel_z",
                       "imu_q_w","imu_q_x", "imu_q_y","imu_q_z",
                       "imu_ori_cov_1","imu_ori_cov_2", "imu_ori_cov_3",
                       "odom_x", "odom_y",
                       "odom_ori_w", "odom_ori_x","odom_ori_y", "odom_ori_z",
                       "odom_vel_x", "odom_vel_y","odom_angular_vel_z",
                       "odom_pose_cov_1", "odom_pose_cov_2", "odom_pose_cov_3",
                       "odom_twist_cov_1","odom_twist_cov_2");

        int status;
        uint64_t gps_timestamp, localTimestamp;
        float latitude, longitude, altitude;
        float East_vel, North_vel, Up_vel;
        double gpsOrientation[3];
        double gpsPosCovariance[3];
        double gpsVelCovariance[3];
        double acceleration[3];
        double angularVelocity[3];
        double orientation[4];
        double orientationCovariance[3];
        double odom[2];
        double odomOrientation[4],odomVelocity[3];
        double odomOrientationCovariance[3], odomTwistCovariance[2];
        bool test = false;
        double origin_x, origin_y;
        bool origin = true;
        while(in.read_row(status, gps_timestamp, localTimestamp,
                          latitude, longitude, altitude,
                          East_vel, North_vel, Up_vel,
                          gpsOrientation[0],gpsOrientation[1],gpsOrientation[2],
                          gpsPosCovariance[0],gpsPosCovariance[1],gpsPosCovariance[2],
                          gpsVelCovariance[0],gpsVelCovariance[1],gpsVelCovariance[2],
                          acceleration[0],acceleration[1],acceleration[2],
                          angularVelocity[0],angularVelocity[1],angularVelocity[2],
                          orientation[3],orientation[0],orientation[1],orientation[2],
                          orientationCovariance[0],orientationCovariance[1],orientationCovariance[2],
                          odom[0],odom[1],
                          odomOrientation[3],odomOrientation[0],odomOrientation[1],odomOrientation[2],
                          odomVelocity[0],odomVelocity[1],odomVelocity[2],
                          odomOrientationCovariance[0],odomOrientationCovariance[1],odomOrientationCovariance[2],
                          odomTwistCovariance[0],odomTwistCovariance[1])
                )
        {
            if (!test)
            {
                test = true;
                ros::Time test_ros = ros::Time::now();
//                std::cout<<test_ros<<std::endl;
//                std::cout<<orientation[0]<<", "<<localTimestamp<<std::endl;
            }
            sensor_msgs::Imu imu;
            ros::Time ros_time;
            ros_time.fromNSec(localTimestamp*1000);
            imu.header.stamp = ros_time;
            imu.header.frame_id = "base_link";
            imu.orientation.x = orientation[0];
            imu.orientation.y = orientation[1];
            imu.orientation.z = orientation[2];
            imu.orientation.w = orientation[3];
            imu.orientation_covariance[0] = orientationCovariance[0];
            imu.orientation_covariance[1] = orientationCovariance[1];
            imu.orientation_covariance[2] = orientationCovariance[2];
            imu.orientation_covariance[3] = 0;
            imu.orientation_covariance[4] = 0;
            imu.orientation_covariance[5] = 0;
            imu.orientation_covariance[6] = 0;
            imu.orientation_covariance[7] = 0;
            imu.orientation_covariance[8] = 0;

            imu.angular_velocity.x = angularVelocity[0];
            imu.angular_velocity.y = angularVelocity[1];
            imu.angular_velocity.z = angularVelocity[2];

            imu.linear_acceleration.x = acceleration[0];
            imu.linear_acceleration.y = acceleration[1];
            imu.linear_acceleration.z = acceleration[2];

            bag.write("/bit/oxts/imu", ros_time, imu);

            if (status == 4)
            {
                sensor_msgs::NavSatFix gps;
                gps.header.stamp = ros_time;
                gps.header.frame_id = "base_link";
                gps.latitude = latitude;
                gps.longitude = longitude;
                gps.altitude = altitude;
                bag.write("/bit/oxts/gps",ros_time,gps);

                double utm_x;
                double utm_y;
                std::string utm_zone;
                LLtoUTM(latitude, longitude, utm_y, utm_x, utm_zone);

                if (origin)
                {
                    geo_converter.Reset(latitude, longitude, altitude);
                    origin_x = utm_x;
                    origin_y = utm_y;
                    origin = false;
                }

                double local_E, local_N, local_U;
                geo_converter.Forward(latitude,longitude,altitude,local_E,local_N,local_U);

                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = ros_time;
                odom_trans.header.frame_id = "world";
                odom_trans.child_frame_id = "base_link";
                odom_trans.transform.translation.x = local_E;
                odom_trans.transform.translation.y = local_N;
                odom_trans.transform.translation.z = local_U;
                odom_trans.transform.rotation.x = orientation[0];
                odom_trans.transform.rotation.y = orientation[1];
                odom_trans.transform.rotation.z = orientation[2];
                odom_trans.transform.rotation.w = orientation[3];
                tf2_msgs::TFMessage tf2;
                tf2.transforms.push_back(odom_trans);
                bag.write("/tf",ros_time,tf2);


                nav_msgs::Odometry odometry;
                odometry.header.stamp = ros_time;
                odometry.header.frame_id = "world";
                odometry.child_frame_id = "base_link";
                odometry.pose.pose.position.x = local_E;
                odometry.pose.pose.position.y = local_N;
//            std::cout<<(utm_x - origin_x)<<", "<<(utm_y - origin_y)<<std::endl;
                odometry.pose.pose.position.z = local_U;
                odometry.pose.pose.orientation.x = odomOrientation[0];
                odometry.pose.pose.orientation.y = odomOrientation[1];
                odometry.pose.pose.orientation.z = odomOrientation[2];
                odometry.pose.pose.orientation.w = odomOrientation[3];
//            odometry.twist.twist.linear.x = odomVelocity[0];
//            odometry.twist.twist.linear.y = odomVelocity[1];
//            odometry.twist.twist.angular.z = odomVelocity[2];
                bag.write("/odom",ros_time,odometry);
            }
        }
    }

    return true;
}

//rosbag写入HDL激光雷达数据
bool HDL_data2bag(rosbag::Bag &bag, string path, int fileNum = 1)
{
    for (int i = 0; i < fileNum; ++i)
    {
        std::cout<<"Reading file "<< (i+1)<<std::endl;
        string file_path = path + "HDL/hdl_" + to_string(i+1) + ".csv";
        ifstream fp(file_path);
        string line;
        getline(fp,line);//
        int j = 0;
        while (getline(fp,line))
        {
            j++;
//            std::cout<<"row: "<<j<<std::endl;
            string number1,number2,number3,number4,number5,number6,number7,number8;
            istringstream str(line);
            int last_id = 0, current_id = 0;
            float last_azimuth = 0.0, current_azimuth = 0.0;
            double last_x = 0.0, current_x = 0.0;
            bool first = true;
            ros::Time ros_time;
            pcl::PointCloud<pcl::PointXYZI> points;
            const size_t kMaxNumberOfPoints = 1e6;  // From the Readme of raw files.
            points.clear();
            points.reserve(kMaxNumberOfPoints);
            double timestamp_pcl;

            while (true)
            {
                getline(str, number1, ' ');
                getline(str, number2, ',');
                uint64_t hdl_timestamp = atoll(number2.c_str());
                getline(str, number3, ',');
                current_id = atoi(number3.c_str());
                getline(str, number4, ',');
                current_azimuth = atof(number4.c_str());
                getline(str, number5, ',');
                current_x = atof(number5.c_str());
                getline(str, number6, ',');
                double y = atof(number6.c_str());
                getline(str, number7, ',');
                double z = atof(number7.c_str());
                getline(str, number8, ',');
                int intensity = atoi(number8.c_str());
                if (first)
                {
                    ros_time.fromNSec(hdl_timestamp*1000);
//                    cloud.header.stamp = ros_time;
                    timestamp_pcl = hdl_timestamp / 1000000;
                }
                if ((current_id == last_id) && (current_azimuth==last_azimuth) && (current_x==last_x))
                    break;

                last_id = current_id;
                last_azimuth = current_azimuth;
                last_x = current_x;

                pcl::PointXYZI point;
                point.x = current_x;
                point.y = y;
                point.z = z;
                point.intensity = intensity;
                points.push_back(point);
            }

            points.header.stamp = timestamp_pcl;
            points.header.frame_id = "velodyne";

            sensor_msgs::PointCloud2 pcl2;
            pcl::toROSMsg(points, pcl2);
            pcl2.header.stamp = ros_time;
            pcl2.header.frame_id = "velodyne";
//            std::cout<<"start write\n";
            bag.write("/bit/velodyne_64/PointCloud",ros_time,pcl2);
        }
    }
    return true;
}

//rosbag写入ZED图像数据
bool ZED_image2bag(rosbag::Bag &bag, string path, int imageNum)
{
    string file_path = path + "ZED/";
    string time_path = path + "ZED/imageTimes.csv";
    io::CSVReader<3> in(time_path);
    in.read_header(io::ignore_extra_column,"leftImageTimestamp", "rightImageTimestamp", "localTimestamp");
    for (int i = 0; i < imageNum; ++i)
    {
        char filename[20];
        sprintf(filename,"frame%05d.jpg",i);
        cv::Mat leftImage = cv::imread(file_path+"left/"+filename,cv::IMREAD_GRAYSCALE);
        cv::imshow("image",leftImage);
        cv::waitKey(1);
        cv::Mat rightImage = cv::imread(file_path+"right/"+filename, cv::IMREAD_GRAYSCALE);
        if (leftImage.empty() || rightImage.empty())
        {
            std::cout<<"wrong image path!\n";
            return false;
        }
        uint64_t left_timestamp, right_timestamp, localTimestamp;
        in.read_row(left_timestamp,right_timestamp,localTimestamp);
        ros::Time ros_time;
        ros_time.fromNSec(left_timestamp*1000);

        //compressed image
        std::string encoding = "mono8";
        int bitDepth = sensor_msgs::image_encodings::bitDepth(encoding);
        sensor_msgs::CompressedImage ros_leftImage_msg, ros_rightImage_msg;
        ros_leftImage_msg.header.stamp = ros_time;
        ros_leftImage_msg.header.frame_id = "frame";
        ros_leftImage_msg.header.seq = 0;
        ros_rightImage_msg.header.stamp = ros_time;
        ros_rightImage_msg.header.frame_id = "frame";
        ros_rightImage_msg.header.seq = 0;

        std::vector<int> params;
        params.resize(9, 0);
        params[0] = cv::IMWRITE_JPEG_QUALITY;
        params[1] = 90;
        params[2] = cv::IMWRITE_JPEG_PROGRESSIVE;
        params[3] = 0;
        params[4] = cv::IMWRITE_JPEG_OPTIMIZE;
        params[5] = 0;
        params[6] = cv::IMWRITE_JPEG_RST_INTERVAL;
        params[7] = 0;
        ros_leftImage_msg.format += "; jpeg compressed ";
        ros_rightImage_msg.format += "; jpeg compressed ";

        if ((bitDepth == 8) || (bitDepth == 16))
        {
            // Target image format
            std::string targetFormat;
            if (sensor_msgs::image_encodings::isColor(encoding))
            {
                // convert color images to BGR8 format
                targetFormat = "bgr8";
                ros_leftImage_msg.format += targetFormat;
                ros_rightImage_msg.format += targetFormat;
            }
            cv::imencode(".jpg", leftImage, ros_leftImage_msg.data, params);
            cv::imencode(".jpg", rightImage, ros_rightImage_msg.data, params);
            bag.write("/bit/zed/left/compressed",ros_time,ros_leftImage_msg);
            bag.write("/bit/zed/right/compressed",ros_time,ros_rightImage_msg);
        }

        //Raw image
//        cv_bridge::CvImage ros_leftImage, ros_rightImage;
//        ros_leftImage.image = leftImage;
//        ros_rightImage.image = rightImage;
//        ros_leftImage.encoding = "mono8";
//        ros_rightImage.encoding = "mono8";
//        sensor_msgs::ImagePtr ros_leftImage_msg, ros_rightImage_msg;
//        ros_leftImage_msg = ros_leftImage.toImageMsg();
//        ros_leftImage_msg->header.seq = 0;
//        ros_leftImage_msg->header.stamp = ros_time;
//        ros_leftImage_msg->header.frame_id = "frame";
//
//        ros_rightImage_msg = ros_rightImage.toImageMsg();
//        ros_rightImage_msg->header.seq = 0;
//        ros_rightImage_msg->header.stamp = ros_time;
//        ros_rightImage_msg->header.frame_id = "frame";
//        bag.write("/bit/zed/left/image_raw",ros_time,ros_leftImage_msg);
//        bag.write("/bit/zed/right/image_raw",ros_time,ros_rightImage_msg);
    }
    return true;
}

//rosbag写入ZED imu数据
bool ZED_imu2bag(rosbag::Bag &bag, string path, int fileNum = 1)
{
    for (int i = 0; i < fileNum; ++i)
    {
        string file_path = path + "ZED/imu/imu_" + to_string(i+1) + ".csv";
        io::CSVReader<12> in(file_path);
        in.read_header(io::ignore_extra_column,"imuTimestamp",
                "localTimestamp", "w", "x", "y", "z",
                "acc_x", "acc_y", "acc_z",
                "vel_x", "vel_y", "vel_z");
        uint64_t imuTimestamp, localTimestamp;
        float x, y, z, w;
        float acc_x, acc_y, acc_z;
        float vel_x, vel_y, vel_z;
        while (in.read_row(imuTimestamp, localTimestamp,
                            w, x, y, z,
                            acc_x, acc_y, acc_z,
                            vel_x, vel_y, vel_z))
        {
            ros::Time ros_time;
            ros_time.fromNSec(imuTimestamp*1000);
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros_time;
            imu_msg.header.frame_id = "zed_imu";
            imu_msg.angular_velocity.x = vel_x;
            imu_msg.angular_velocity.y = vel_y;
            imu_msg.angular_velocity.z = vel_z;
            imu_msg.linear_acceleration.x = acc_x;
            imu_msg.linear_acceleration.y = acc_y;
            imu_msg.linear_acceleration.z = acc_z;
            bag.write("/bit/zed/imu", ros_time, imu_msg);
        }
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"multi_data");
    ros::NodeHandle nh("~");
	string csv_path;//csv
    nh.param<std::string>("csv_path",csv_path,"");
    string bag_path;
    nh.param<std::string>("bag_path", bag_path, "");
    bool bOxts, bHdl, bZedImage, bZedImu;
    nh.param<bool>("b_oxts", bOxts, 0);
    nh.param<bool>("b_hdl", bHdl, 0);
    nh.param<bool>("b_zedImage", bZedImage, 0);
    nh.param<bool>("b_zedImu", bZedImu, 0);
    int oxts_dataNum, hdl_dataNum, zed_imageNum, zed_imuNum;
    nh.param<int>("oxts_dataNum", oxts_dataNum,1);
    nh.param<int>("hdl_dataNum", hdl_dataNum,1);
    nh.param<int>("zed_imageNum", zed_imageNum,1);
    nh.param<int>("zed_imuNum", zed_imuNum,1);

    //for test
	//    ROS_INFO("%s",csv_path.c_str());
//    ROS_INFO("%s",bag_path.c_str());
//    ROS_INFO("%d, %d, %d, %d", bOxts, bHdl, bZedImage, bZedImu);
//    ROS_INFO("%d, %d, %d, %d", oxts_dataNum, hdl_dataNum, zed_imageNum, zed_imuNum);
    /*ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud",
            1,&msg_callback);
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;*/
    rosbag::Bag bag;
    bag.open(bag_path,rosbag::bagmode::Write);
    bool success = true;

	//OXTS
    if (bOxts)
    {
        ROS_INFO("OXTS data is writing!");
        success = OXTS_data2bag(bag, csv_path, oxts_dataNum);
        if(!success)
            std::cout<<"OXTS data failed!\n";
    }
    //HDL
    if (bHdl)
    {
        ROS_INFO("HDL data is writing!");
        success = HDL_data2bag(bag, csv_path, hdl_dataNum);
        if(!success)
            std::cout<<"HDL data failed!\n";
    }

    //ZED Image
    if (bZedImage)
    {
        ROS_INFO("ZED Image is writing!");
        success = ZED_image2bag(bag,csv_path,zed_imageNum);
        if(!success)
            std::cout<<"ZED image failed!\n";
    }

    //ZED imu
    if (bZedImu)
    {
        ROS_INFO("ZED imu is writing!");
        success = ZED_imu2bag(bag,csv_path, zed_imuNum);
        if(!success)
            std::cout<<"ZED imu failed!\n";
    }
    bag.close();
	return 0;

}