#pragma once

#include <ros/ros.h>

// 多消息同步回调
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ROS_KCF/kcftracker.hpp"
#include "eco/include/eco.hpp"

#include "AAMED/FLED.h"

#include "DBSCAN/dbscan.h"

namespace CADC{

struct classifter
{
    int index = -1;
    double average_depth = 0;
};

class DepthCamera
{
private:
    ros::Publisher position;
    ros::Publisher aamed_pub;

    ros::Subscriber local_pos_sub;
    ros::Subscriber state;
    ros::Subscriber odom_sub;
    ros::Subscriber img_sub;
    ros::Subscriber cam_info_sub;


    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy_img_depth;

    ros::NodeHandle nh;

    int ROWS,COLS;

    double body_offset_x_, body_offset_y_, height_;

    const double baseline = 0.06;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;  // left Camera intrinsics
    double k1_,k2_,p1_,p2_,k3_;
    Eigen::Matrix<double, 5, 1> D_;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();  // 内参矩阵
    Eigen::Matrix3d K_1_ = Eigen::Matrix3d::Identity();  // 内参矩阵的逆
    Eigen::Matrix3d R_b_bc_ = Eigen::Matrix3d::Identity();  // 相机坐标转换成机体坐标系
    Eigen::Matrix3d R_Lc_bc_ = Eigen::Matrix3d::Identity();   // 相机坐标系转换成相机的水平坐标系

    // KCF 跟踪器
    KCFTracker tracker;     double centerDistance;
    // ECO 跟踪器
    eco::ECO ecotracker;    eco::EcoParameters parameters;

    AAMED aamed;

    DBSCAN::DBSCAN ds;
    DBSCAN::DBSCAN ds_1d;

    void init_publisher();
    void init_subscriber();
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_cb(const std_msgs::Bool::ConstPtr& msg);

    void CompressImg_cb(const sensor_msgs::CompressedImageConstPtr& img_compress);
    void Img_cb(const sensor_msgs::ImageConstPtr& img_Ptr);
    void img_depth_cb(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::ImageConstPtr &dep);
    void cam_info_cb(const sensor_msgs::CameraInfo::ConstPtr& cam_left_Info_ptr);
    void odomCallback(const nav_msgs::OdometryConstPtr &odom);
public: 
    Eigen::Matrix3d R_Lb_ = Eigen::Matrix3d::Identity();  // 机体坐标系旋转到水平坐标系

public:
    DepthCamera(ros::NodeHandle* nodehandle, 
            AAMED aam, KCFTracker tra, DBSCAN::DBSCAN d, DBSCAN::DBSCAN d_1,
            double centerDistance, double body_offset_x, double body_offset_y, double height);
    ~DepthCamera(); 

    ros::Timer calc_timer;

    virtual void calc_cb(const ros::TimerEvent&);

    void Body2LevelRotationMatrix(const Eigen::Vector3d &q);
    
    bool classifiterPointSet(const std::vector<cv::RotatedRect> &detEllipses, DBSCAN::DBSCAN &ds);
    
    bool classifiter1DpointSet(std::vector<cv::Point> &point, DBSCAN::DBSCAN &ds);

    bool classifiter1DpointSet(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &point, DBSCAN::DBSCAN &ds);

    void ellipseFilter(DBSCAN::DBSCAN &ds);

    double detectDepthOfEllipse(const cv::RotatedRect &RRect, const cv::Rect &Rect, const cv::Mat &disparity, cv::Mat &rgb);

    std::vector<cv::Point> pixelOfEllipseGrad(const cv::RotatedRect &RRect, const cv::Rect &Rect,const cv::Mat &depth);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> point3dOfEllipseGrad(const cv::RotatedRect &RRect, const cv::Rect &Rect,const cv::Mat &depth);

    double getDepth(const cv::Mat &depth, int x, int y);

    bool Point3d(const cv::Mat &depth, const Eigen::Vector2d &pixel, Eigen::Vector3d &point);

    Eigen::Vector2d distortedPixel(const Eigen::Vector2d pixel);

    bool choosePoint(const std::vector<classifter> &classifter);

    double computeArea(const cv::RotatedRect &rotaterect, const CADC::classifter &c);

    cv::Rect2f computeEllipseRoI(const cv::RotatedRect &rotaterect);

    bool CenterDistance(const cv::Rect &r1, const cv::Rect &r2);

    double distance2d(const cv::Point2d p1, const cv::Point2d p2);


    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped point_aim;

    std_msgs::Bool detect_sign;
    std_msgs::Bool point_aim_;
    std::vector<DBSCAN::Point> point;

    cv::Mat img_raw, img_g, depth;

    sensor_msgs::CameraInfo cam_info;
    Eigen::Vector3d eular;
    
    std::vector<classifter> cfter;
    cv::RotatedRect aim_rrect;
    classifter min_area;

    cv::Rect2f tracker_result;
    cv::Rect2f initRoi;
    cv::Rect2f nowRoi;
    uint count = 1;
    uint frame = 0;
    uint ellipse_size_l = 0;

    ros::Time last;
};


}