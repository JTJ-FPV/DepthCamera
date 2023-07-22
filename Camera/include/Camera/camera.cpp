#include "camera.h"
#include <cmath>

namespace CADC
{
DepthCamera::~DepthCamera()
{
    aamed.~FLED();
    ecotracker.~ECO();
    ds.~DBSCAN();
    ds_1d.~DBSCAN();
    point.~vector();
    cfter.~vector();
}

DepthCamera::DepthCamera(ros::NodeHandle* nodehandle, 
                AAMED aam, KCFTracker tra, DBSCAN::DBSCAN d,DBSCAN::DBSCAN d_1,
                double centerDistan, double body_offset_x = 0, double body_offset_y = 0, double height = 0)
                :nh(*nodehandle), aamed(aam), ds(d), ds_1d(d_1), centerDistance(centerDistan)
{   
    body_offset_x_ = body_offset_x, body_offset_y_ = body_offset_y, height_ = height; 
    // 设置椭圆识别参数
    aamed.SetParameters(CV_PI / 3, 3.4, 0.77);

    ROS_INFO("init_publisher before");
    init_publisher();
    ROS_INFO("init_publisher after");
    ROS_INFO("init_subscriber before");
    init_subscriber();
    ROS_INFO("init_subscriber after");
    calc_timer = nh.createTimer(ros::Duration(0.05), &DepthCamera::calc_cb, this);
    COLS = aamed.getdCOLS();
    ROWS = aamed.getdROWS();


    // 初始化 ECO 跟踪器的参数
    /*******************Good********************/
    parameters.useDeepFeature = true;
    parameters.useHogFeature = true;
    parameters.max_score_threshhold = 0.3;
    parameters.useIcFeature = true;
    /*******************Good********************/

    // *************simulation*************
    // R_b_bc_ << 0, -1, 0,
    //            -1, 0, 0,
    //            0, 0, -1;
    // *************simulation*************

    // *************realflight*************
    R_b_bc_ << 1, 0, 0,
               0, -1, 0,
               0, 0, -1;
    // *************realflight*************
    ROS_INFO("Construct");
}


void DepthCamera::init_publisher()
{
    position = nh.advertise<geometry_msgs::PoseStamped>
        ("/camera/ellipse/center", 10, this);
    aamed_pub = nh.advertise<std_msgs::Bool>
        ("/camera/ellipse/Bool", 10, this);
}

void DepthCamera::init_subscriber()
{
    // *************simulation*************
    // local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //     ("/mavros/local_position/pose", 10, &DepthCamera::local_pos_cb, this);
    
    // static message_filters::Subscriber<sensor_msgs::Image> img_color_raw(nh, "/iris/realsense/depth_camera/color/image_raw", 1);
    // static message_filters::Subscriber<sensor_msgs::Image> align_color_depth(nh, "/iris/realsense/depth_camera/depth/image_raw", 1);
    // static message_filters::Synchronizer<MySyncPolicy_img_depth> sync(MySyncPolicy_img_depth(10), img_color_raw, align_color_depth);
    // sync.registerCallback(boost::bind(&DepthCamera::img_depth_cb, this, _1, _2));

    // cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>
    //     ("/iris/realsense/depth_camera/color/camera_info", 10, &DepthCamera::cam_info_cb, this);
    // *************simulation*************
    

    // *************realflight*************
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/vision_pose/pose", 10, &DepthCamera::local_pos_cb, this);

    static message_filters::Subscriber<sensor_msgs::Image> img_color_raw(nh, "/camera/color/image_raw", 1);
    static message_filters::Subscriber<sensor_msgs::Image> align_color_depth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    static message_filters::Synchronizer<MySyncPolicy_img_depth> sync(MySyncPolicy_img_depth(10), img_color_raw, align_color_depth);
    sync.registerCallback(boost::bind(&DepthCamera::img_depth_cb, this, _1, _2));

    cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>
        ("/camera/color/camera_info", 10, &DepthCamera::cam_info_cb, this);
    
    // *************realflight*************
    

    state = nh.subscribe<std_msgs::Bool>
        ("/cadc_controll/detect", 10, &DepthCamera::state_cb, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("/mavros/local_position/odom", 1, &DepthCamera::odomCallback, this);
}

void DepthCamera::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;
}

void DepthCamera::state_cb(const std_msgs::Bool::ConstPtr& msg)
{
    detect_sign = *msg;
}

void DepthCamera::img_depth_cb(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::ImageConstPtr &dep)
{
    try{
        cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        // *************simulation*************
        // cv_bridge::CvImagePtr dep_ptr = cv_bridge::toCvCopy(dep, sensor_msgs::image_encodings::TYPE_32FC1);
        // *************simulation*************

        // *************realflight*************
        cv_bridge::CvImagePtr dep_ptr = cv_bridge::toCvCopy(dep, sensor_msgs::image_encodings::TYPE_16UC1);
        // *************realflight*************
        img_raw = img_ptr->image;
        depth = dep_ptr->image;
        // ROS_INFO_STREAM("Center is :" << depth.at<uchar>(COLS / 2, ROWS / 2) );
        ROS_INFO_STREAM("Center is :" << getDepth(depth, COLS/2, ROWS/2) );
        cv::cvtColor(img_raw, img_g, cv::COLOR_RGB2GRAY);
        aamed.run_FLED(img_g);
        bool cp = false;
        if(!aamed.detEllipses.empty() && classifiterPointSet(aamed.detEllipses, ds))
        {
            ellipseFilter(ds);
            cp = choosePoint(cfter);
            if(cp && 0 == frame)
            {
                initRoi = computeEllipseRoI(aim_rrect);
                ecotracker.init(img_raw, initRoi, parameters);
                cv::rectangle(img_raw, initRoi, cv::Scalar(0, 255, 255), 3);
                ++frame;
            }
        }
        if(0 != frame)
        {
            bool okeco = ecotracker.update(img_raw, nowRoi);
            if(okeco)   // 跟踪成功
            {
                cv::rectangle(img_raw, nowRoi, cv::Scalar(0, 255, 255), 3);
                ++frame;
                if(cp)
                {
                    // double area = computeArea(aim_rrect, min_area);
                    // 判断该椭圆是否在区域内
                    if(nowRoi.contains(cv::Point2f(aamed.detEllipses.at(min_area.index).center.y, 
                                                   aamed.detEllipses.at(min_area.index).center.x)))
                    {
                        double area = computeArea(aim_rrect, min_area);
                        // vector<int> index;
                        int index = 0, count = 0;;
                        double Max = 0;
                        for(const auto &ellipse : aamed.detEllipses)
                        {
                            if(nowRoi.contains(cv::Point2f(ellipse.center.y, ellipse.center.x)))
                            {
                                double a = computeArea(ellipse, min_area);
                                if(a > Max)
                                {
                                    Max = a;
                                    index = count;
                                }
                            }
                            ++count;
                        }
                        point_aim_.data = true;
                        aamed_pub.publish(point_aim_);
                        last = ros::Time::now();
                    }
                    else if(ros::Time::now() - last > ros::Duration(1))     // 识别区域暂时没有检测到椭圆，如果1S之后仍未检测到椭圆则重新初始化
                        frame == 0;

                }
                else if(ros::Time::now() - last > ros::Duration(3)) // 整张图片没有检测到椭圆，3S后若仍未检测到椭圆则重新初始化
                    frame = 0;
            }
            else    // 跟踪失败，直接初始化
                frame = 0;
            
        }
        aamed.drawFLED(img_raw, "", "IMGAE COLOR VIEWER");
        // cv::imshow("IMGAE COLOR VIEWER", img_raw);
        cv::imshow("IMGAE DEPTH VIEWER", depth);
    }
    catch(cv_bridge::Exception &e){
        ROS_ERROR("convert fail!");
    }

}

void DepthCamera::cam_info_cb(const sensor_msgs::CameraInfo::ConstPtr& cam_Info_ptr)
{
    if(1 == count)
    {
        ROS_INFO("INIT LEFT intrinsics");
        cam_info = *cam_Info_ptr;
        fx_ = cam_info.K.at(0);
        fy_ = cam_info.K.at(4);
        cx_ = cam_info.K.at(2);
        cy_ = cam_info.K.at(5);
        K_ << fx_, 0, cx_,
                0, fy_, cy_,
                0, 0, 1;
        K_1_ = K_.inverse();

        k1_ = cam_info.D.at(0);
        k2_ = cam_info.D.at(1);
        p1_ = cam_info.D.at(2);
        p2_ = cam_info.D.at(3);
        k3_ = cam_info.D.at(4);
        D_ << k1_, k2_, p1_, p2_, k3_; 
        count = 2;
    }
}

void DepthCamera::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);

    
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    eular << roll, pitch, yaw;
    // ROS_INFO("*************************************");
    // ROS_INFO_STREAM("ROLL : " << roll * 180 / M_PI);
    // ROS_INFO_STREAM("PITCH : " << pitch * 180 / M_PI);
    // ROS_INFO_STREAM("YAW : " << yaw * 180 / M_PI);
    // ROS_INFO("*************************************");
    Body2LevelRotationMatrix(eular);
}

void DepthCamera::Body2LevelRotationMatrix(const Eigen::Vector3d &q)
{  
    R_Lb_ = Eigen::AngleAxisd(-q(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-q(0), Eigen::Vector3d::UnitX());
    R_Lc_bc_ = Eigen::AngleAxisd(q(0), Eigen::Vector3d::UnitY()) * 
               Eigen::AngleAxisd(q(1), Eigen::Vector3d::UnitX());
    // ROS_INFO("*************************************");
    // Eigen::AngleAxisd a(R_Lb_);
    // Eigen::Vector3d v = a.matrix().eulerAngles(2, 1, 0);          // ZYX顺序,roll, pitch, yaw
    // ROS_INFO("Level Coordinate X AIXED : %lf", v(2) * 180 / M_PI);
	// ROS_INFO("Level Coordinate Y AIXED : %lf", v(1) * 180 / M_PI);
	// ROS_INFO("Level Coordinate Z AIXED : %lf", v(0) * 180 / M_PI);
    // ROS_INFO("*************************************");
}

bool DepthCamera::classifiterPointSet(const std::vector<cv::RotatedRect> &detEllipses, DBSCAN::DBSCAN &ds)
{
    if(detEllipses.empty())
        return false;
    point.clear();
    ds.m_points.clear();
    DBSCAN::Point p;
    for(int i = 0; i < detEllipses.size(); ++i)
    {
        p.x = detEllipses[i].center.y;
        p.y = detEllipses[i].center.x;
        // p.z = 1 / detEllipses[i].size.width + 1 / detEllipses[i].size.height + detEllipses[i].angle;
        p.index_ellipse = i;
        p.clusterID = UNCLASSIFIED;
        ds.m_points.push_back(p);
    }
    ds.run();
    if(0 == ds.getClusterPoint())
        return false;
    return true;
}

bool DepthCamera::classifiter1DpointSet(std::vector<cv::Point> &point, DBSCAN::DBSCAN &ds)
{
    if(point.empty())
        return false;
    ds.m_points.clear();
    DBSCAN::Point p;
    for(int i = 0; i < point.size(); ++i)
    {
        p.x = 0;
        p.y = 0;
        p.z = getDepth(depth, point.at(i).x, point.at(i).y);
        p.index_ellipse = i;
        p.clusterID = UNCLASSIFIED;
        ds.m_points.push_back(p);
    }
    ds.run();
    if(0 == ds.getClusterPoint())
        return false;
    return true;
}

bool DepthCamera::classifiter1DpointSet(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &point, DBSCAN::DBSCAN &ds)
{
    if(point.empty())
        return false;
    ds.m_points.clear();
    DBSCAN::Point p;
    for(int i = 0; i < point.size(); ++i)
    {
        p.x = 0;
        p.y = 0;
        p.z = point.at(i).z();
        // p.z = point.at(i).z();
        // p.x = point.at(i).x();
        // p.y = point.at(i).y();
        p.index_ellipse = i;
        p.clusterID = UNCLASSIFIED;
        ds.m_points.push_back(p);
    }
    ds.run();
    if(0 == ds.getClusterPoint())
        return false;
    return true;
}

void DepthCamera::ellipseFilter(DBSCAN::DBSCAN &ds)
{
    cfter.clear();
    cv::RotatedRect rrect;
    double level_average_depth = 0;
    CADC::classifter c;
    for(int i = 0; i < ds.m_points.size(); ++i)
    {
        ROS_INFO_STREAM("ds.m_points[" << i << "].clusterID = " << ds.m_points[i].clusterID);
    }
    for(int j = 1; j <= ds.getClusterPoint(); ++j)
    {
        for(int i = 0; i < ds.m_points.size(); ++i)
        {
            bool sign = false;
            if(j == ds.m_points[i].clusterID)
            {
                sign = true;
            }
            if(sign)
            {
                rrect.center.x = aamed.detEllipses[i].center.y;
                rrect.center.y = aamed.detEllipses[i].center.x;
                rrect.angle = -aamed.detEllipses[i].angle;
                rrect.size.width = aamed.detEllipses[i].size.height;
                rrect.size.height = aamed.detEllipses[i].size.width;
                level_average_depth = detectDepthOfEllipse(rrect, computeEllipseRoI(aamed.detEllipses[i]), depth, img_raw);
                ROS_INFO_STREAM("average depth is : " << level_average_depth);
                // if(level_average_depth > 0.1 && level_average_depth <= pose.pose.position.z - 0.05)
                if(level_average_depth > 0.1 && level_average_depth <= 0.7)
                {
                    c.index = i;
                    c.average_depth = level_average_depth;
                    cfter.push_back(c);
                    ROS_INFO_STREAM("the " << i << " Ellipse is push back");
                    // ROS_INFO_STREAM("pose.pose.position.z - 2 :" << pose.pose.position.z - 2 << " " <<i);
                }
            }
        }
    }
    ROS_INFO_STREAM("the classifer size is " << cfter.size());
}

// 返回的是水平坐标系下椭圆的深度值，并非机体当前的坐标下椭圆的深度值
double DepthCamera::detectDepthOfEllipse(const cv::RotatedRect &RRect, const cv::Rect &Rect, const cv::Mat &depth, cv::Mat &rgb)
{
    // std::vector<cv::Point> point = pixelOfEllipseGrad(RRect, Rect, depth);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> point_3d = point3dOfEllipseGrad(RRect, Rect, depth);
    // if(classifiter1DpointSet(point, ds_1d))
    if(classifiter1DpointSet(point_3d, ds_1d))
    {
        ROS_INFO_STREAM("DBSCAN 1 DIMENSION CLUSTER " << ds_1d.getClusterPoint());
        double d = 0;
        int count = 0;
        std::vector<double> dep;
        bool sign = false;
        for(int j = 1; j <= ds_1d.getClusterPoint(); ++j)
        {
            d = 0, count = 0;
            sign = false;
            for(int i = 0; i < ds_1d.m_points.size(); ++i)
            {
                if(j == ds_1d.m_points.at(i).clusterID)
                {
                    d += ds_1d.m_points.at(i).z;
                    ++count;
                    sign = true;
                }
            }
            if(sign)
            {
                d /= count;
                ROS_INFO_STREAM("THE DBSCAN 1 DIMENSION d is " << d);
                dep.push_back(d);
            }
        }
        std::sort(dep.begin(), dep.end(), std::greater<double>());  // 降序排列
        // for(int i = 0; i < dep.size(); ++i)
        //     ROS_INFO_STREAM("dep[" << i << ']' << '=' << dep.at(i));
        while (dep.back() < 0.1 && dep.size() > 1)
        {
            dep.pop_back();
        }   
        ROS_INFO_STREAM("depth : " << dep.back());
        return dep.back();
    }
    return 0;
}

std::vector<cv::Point> DepthCamera::pixelOfEllipseGrad(const cv::RotatedRect &RRect, const cv::Rect &Rect,const cv::Mat &depth)
{
    vector<cv::Point> point, p;
    // 获取粗糙的椭圆像素坐标
    cv::ellipse2Poly(RRect.center, cv::Size2d(RRect.size.width / 2, RRect.size.height / 2), RRect.angle, 0, 360, 3, point);

    // 使用最小二乘法迭代求解椭圆像素坐标
    // 确定搜索窗口大小
    int half_patch_size = 4;
    // 迭代次数
    int iterations = 5;
    double cost = 0, lastCost = 0;
    // 深度图像的梯度
    float depth_x = 0, depth_y = 0, depth_xx = 0, depth_yy = 0;
    Eigen::Vector2d X;
    bool succ;
    cv::Point pixel;
    for(size_t i = 0; i < point.size(); ++i)
    {
        X << point[i].x, point[i].y;
        succ = true;
        for(int iter = 0; iter < iterations; ++iter)
        {
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        cost = 0, depth_x = 0, depth_y = 0, depth_xx = 0, depth_yy = 0;

        if(point[i].x - half_patch_size < 0 || point[i].y - half_patch_size < 0 ||
        point[i].x + half_patch_size > COLS -1 || point[i].y + half_patch_size > ROWS - 1)
        {
            succ = false;
            break;
        }      
        for(int x = -half_patch_size; x < half_patch_size; ++x)
            for(int y = -half_patch_size; y < half_patch_size; ++y)
            {
            depth_x = getDepth(depth, point[i].x + x + 1, point[i].y + y) - getDepth(depth, point[i].x + x, point[i].y + y);
            depth_y = getDepth(depth, point[i].x + x, point[i].y + y + 1) - getDepth(depth, point[i].x + x, point[i].y + y);
            depth_xx = getDepth(depth, point[i].x + x + 2, point[i].y + y) - 2 * getDepth(depth, point[i].x + x + 1, point[i].y + y) + getDepth(depth, point[i].x + x, point[i].y + y);
            depth_yy = getDepth(depth, point[i].x + x, point[i].y + y + 2) - 2 * getDepth(depth, point[i].x + x, point[i].y + y + 1) + getDepth(depth, point[i].x + x, point[i].y + y);
            double error = 0;
            Eigen::Vector2d J;   // Jcaobian
            // 残差计算
            error += pow(depth_x, 2) + pow(depth_y, 2);
            // 雅可比
            J = 2 * Eigen::Vector2d(depth_x * depth_xx, depth_y * depth_yy);

            H += J * J.transpose();
            b += -J * error;
            // ROS_INFO_STREAM("H is :" << H);
            // ROS_INFO_STREAM("J is :" << J );
            cost += error * error;
            }
        // 求解线性方程 udpate = H^-1 * b
        Eigen::Vector2d update;
        update = H.ldlt().solve(b);

        if(isnan(update[0]))
        {
            succ = false;
            ROS_INFO("result is nan!");
            break;
        }
        
        if(iter > 0 && cost > lastCost)
        {
            ROS_INFO("cost increased: %lf", lastCost);
            break;
        }
        X += update;
        lastCost = cost;
        succ = true;
        }

        pixel.x = round(X[0]);
        pixel.y = round(X[1]);
        if(succ && Rect.contains(pixel))
        p.push_back(pixel);
    }

    return p;
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> DepthCamera::point3dOfEllipseGrad(const cv::RotatedRect &RRect, const cv::Rect &Rect,const cv::Mat &depth)
{
    vector<cv::Point> point;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> p;
    // 获取粗糙的椭圆像素坐标
    cv::ellipse2Poly(RRect.center, cv::Size2d(RRect.size.width / 2, RRect.size.height / 2), RRect.angle, 0, 360, 3, point);

    // 使用最小二乘法迭代求解椭圆像素坐标
    // 确定搜索窗口大小  4
    int half_patch_size = 3;
    // 迭代次数    5
    int iterations = 2;
    double cost = 0, lastCost = 0;
    // 深度图像的梯度
    float depth_x = 0, depth_y = 0, depth_xx = 0, depth_yy = 0;
    Eigen::Vector2d X;
    Eigen::Vector3d X_Body, x_Level;
    bool succ;
    cv::Point pixel;
    for(size_t i = 0; i < point.size(); ++i)
    {
        X << point[i].x, point[i].y;
        succ = true;
        for(int iter = 0; iter < iterations; ++iter)
        {
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        cost = 0, depth_x = 0, depth_y = 0, depth_xx = 0, depth_yy = 0;

        if(point[i].x - half_patch_size < 0 || point[i].y - half_patch_size < 0 ||
        point[i].x + half_patch_size > COLS -1 || point[i].y + half_patch_size > ROWS - 1)
        {
            succ = false;
            break;
        }      
        for(int x = -half_patch_size; x < half_patch_size; ++x)
            for(int y = -half_patch_size; y < half_patch_size; ++y)
            {
            depth_x = getDepth(depth, point[i].x + x + 1, point[i].y + y) - getDepth(depth, point[i].x + x, point[i].y + y);
            depth_y = getDepth(depth, point[i].x + x, point[i].y + y + 1) - getDepth(depth, point[i].x + x, point[i].y + y);
            depth_xx = getDepth(depth, point[i].x + x + 2, point[i].y + y) - 2 * getDepth(depth, point[i].x + x + 1, point[i].y + y) + getDepth(depth, point[i].x + x, point[i].y + y);
            depth_yy = getDepth(depth, point[i].x + x, point[i].y + y + 2) - 2 * getDepth(depth, point[i].x + x, point[i].y + y + 1) + getDepth(depth, point[i].x + x, point[i].y + y);
            double error = 0;
            Eigen::Vector2d J;   // Jcaobian
            // 残差计算
            error += pow(depth_x, 2) + pow(depth_y, 2);
            // 雅可比
            J = 2 * Eigen::Vector2d(depth_x * depth_xx, depth_y * depth_yy);

            H += J * J.transpose();
            b += -J * error;
            // ROS_INFO_STREAM("H is :" << H);
            // ROS_INFO_STREAM("J is :" << J );
            cost += error * error;
            }
        // 求解线性方程 udpate = H^-1 * b
        Eigen::Vector2d update;
        update = H.ldlt().solve(b);

        if(isnan(update[0]))
        {
            succ = false;
            ROS_INFO("result is nan!");
            break;
        }
        
        if(iter > 0 && cost > lastCost)
        {
            ROS_INFO("cost increased: %lf", lastCost);
            break;
        }
        X += update;
        lastCost = cost;
        succ = true;
        }


        pixel.x = round(X[0]);
        pixel.y = round(X[1]);
        if(succ && Rect.contains(pixel))
        {
            X_Body << pixel.x, pixel.y, 1;
            if(Point3d(depth, X, X_Body))
            {    // 获取水平坐标
                x_Level = R_Lb_ * X_Body;
                p.push_back(X_Body);
            }
        }
    }

    return p;
}

double DepthCamera::getDepth(const cv::Mat &depth, int x, int y)
{
    // ushort d = depth_pic.at<ushort>(depth_pic.rows/2,depth_pic.cols/2);
    /*
        再作一步范围检测以防出现溢出
    */
    if(x < 0)
        x = 0;
    else if(x > COLS)
        x = COLS - 1;
    else if(y < 0)
        y = 0;
    else if(y > ROWS)
        y = ROWS - 1;
    ushort d = depth.at<ushort>(y, x);  

    // *************realflight*************
    return double(d)/1000 ;      //强制转换
    // *************realflight*************

    // *************simulation*************
    // return double(d)/10000 ;      //强制转换
    // *************simulation*************
}

bool DepthCamera::Point3d(const cv::Mat &depth, const Eigen::Vector2d &pixel, Eigen::Vector3d &point)
{
    double x, y, d;
    if (getDepth(depth, pixel(0), pixel(1)) <= 0)
        return false;
    x = (pixel(0) - cx_) / fx_;
    y = (pixel(1) - cy_) / fy_;
    d = getDepth(depth, pixel(0), pixel(1));
    // 去畸变
    Eigen::Vector2d pixel_distorted = distortedPixel(Eigen::Vector2d(x, y));
    point << pixel_distorted(0), pixel_distorted(1) , 1;
    point = d * point;
    return true;
}

Eigen::Vector2d DepthCamera::distortedPixel(const Eigen::Vector2d pixel)
{
    Eigen::Vector2d pixel_distorted;
    double r = sqrt(pixel.transpose() * pixel);
    pixel_distorted(0) = pixel(0) * (1 + k1_ * r * r + k2_ * r * r * r * r)
            + 2 * p1_ * pixel(0) * pixel(1) + p2_ * (r * r + 2 * pixel(0) * pixel(0));
    pixel_distorted(1) = pixel(1) * (1 + k1_ * r * r + k2_ * r * r * r * r)
            + p1_ * (r * r + 2 * pixel(1) * pixel(1)) + 2 * p2_ * pixel(0) * pixel(1);
    return pixel_distorted;
}

bool DepthCamera::choosePoint(const std::vector<CADC::classifter> &classifter)
{
    // 从左目椭圆索引中找到面积最小的椭圆
    if(classifter.empty())
        return false;
    double area;
    double Min_area = computeArea(aamed.detEllipses[classifter[0].index], classifter[0]);
    int count = 0;
    for(int i = 0;i < classifter.size(); ++i)
    {   
        area = computeArea(aamed.detEllipses[classifter[i].index], classifter[i]);
        ROS_INFO_STREAM("the AREA is " << area);
        if(area < Min_area)
        {
            Min_area = area;
            count = i;
        }    
    }
    aim_rrect = aamed.detEllipses[classifter[count].index];
    min_area.average_depth = classifter[count].average_depth;
    min_area.index = classifter[count].index;
    cv::Point2d center(aamed.detEllipses[classifter[count].index].center.y, aamed.detEllipses[classifter[count].index].center.x);
    cv::circle(img_raw, center, 3, cv::Scalar(0, 255, 0), -1);
    Eigen::Vector3d aim;
    Eigen::Vector2d pixel(aamed.detEllipses[count].center.y, aamed.detEllipses[count].center.x);
    /*R_r_b_bc_ * */ 
    if(Point3d(depth, pixel, aim))
    {
        ROS_INFO_STREAM("stereo point : " << '(' << aim(0) << ", " << aim(1) << ", " << aim(2) << ')');
        point_aim.pose.position.x = aim(0);
        point_aim.pose.position.y = aim(1);
        point_aim.pose.position.z = aim(2);
        return true;
    }
    return false;
}

double DepthCamera::computeArea(const cv::RotatedRect &rotaterect, const CADC::classifter &c)
{
    cv::RotatedRect temp;
    Point2f ver[4];
    temp.size.height = rotaterect.size.width;
    temp.size.width = rotaterect.size.height;
    temp.center.x = rotaterect.center.y;
    temp.center.y = rotaterect.center.x;
    temp.angle = -rotaterect.angle;
    temp.points(ver);
    Eigen::Vector2d U1_d(ver[0].x, ver[0].y);
    Eigen::Vector2d U2_d(ver[1].x, ver[1].y);
    Eigen::Vector2d U3_d(ver[2].x, ver[2].y);
    U1_d = distortedPixel(U1_d);
    U2_d = distortedPixel(U2_d);
    U3_d = distortedPixel(U3_d);
    Eigen::Vector3d U1(U1_d(0), U1_d(1), 1);
    Eigen::Vector3d U2(U2_d(0), U2_d(1), 1);
    Eigen::Vector3d U3(U3_d(0), U3_d(1), 1);
    // double Z1 = getDepth(depth, U1(0), U1(1));
    // double Z2 = getDepth(depth, U2(0), U2(1));
    // double Z3 = getDepth(depth, U3(0), U3(1));
    double Z1 = c.average_depth;
    double Z2 = Z1;
    double Z3 = Z1;
    double a_2 = 0.25 * (Z2 * U2 - Z1 * U1).transpose() * K_1_.transpose() * K_1_ * (Z2 * U2 - Z1 * U1);
    double b_2 = 0.25 * (Z3 * U3 - Z2 * U2).transpose() * K_1_.transpose() * K_1_ * (Z3 * U3 - Z2 * U2);
    return sqrt(a_2 * b_2);
}

cv::Rect2f DepthCamera::computeEllipseRoI(const cv::RotatedRect &rotaterect)
{
    cv::RotatedRect temp;
    Point2f ver[4];
    temp.size.height = rotaterect.size.width;
    temp.size.width = rotaterect.size.height;
    temp.center.x = rotaterect.center.y;
    temp.center.y = rotaterect.center.x;
    temp.angle = -rotaterect.angle;
    temp.points(ver);
    int radius =  (int)(sqrt(pow((ver[0].x - ver[2].x), 2) + pow(ver[0].y - ver[2].y, 2)) * 0.5);
    int right_up_x = temp.center.x - radius;
    int right_up_y = temp.center.y - radius;
    int r_x = radius * 2, r_y = radius * 2;
    if(right_up_x < 0 && right_up_y < 0 && right_up_x > -2 * radius && right_up_y > -2 * radius){
    r_x = 2 * radius + right_up_x;
    r_y = 2 * radius + right_up_y;
    right_up_x = 0, right_up_y = 0;
                    // cout << 1 << endl;
    }
    else if(right_up_x < 0 && right_up_y < 0 && (right_up_x <= -2 * radius || right_up_y <= -2 * radius)){
        r_x = 2 * radius + right_up_x;
        r_y = 2 * radius + right_up_y;
        right_up_x = 0, right_up_y = 0;
        // cout << 2 << endl;
    }
    else if(right_up_x < 0 && right_up_y >= 0 && right_up_x > -2 * radius && right_up_y < ROWS - 2 * radius){
        r_x = 2 * radius + right_up_x;
        right_up_x = 0;
        // cout << 3 << endl;
    }
    else if(right_up_x < 0 && right_up_y >= 0 && right_up_x > -2 * radius && right_up_y >= ROWS - 2 * radius && right_up_y < ROWS){
        r_x = 2 * radius + right_up_x;
        r_y= ROWS - right_up_y - 1;
        right_up_x = 0;
        // cout << 4 << endl;
    }
    else if(right_up_x < 0 && right_up_y >= 0 && (right_up_x <= -2 * radius || right_up_y >= ROWS)){
        r_x = 2 * radius + right_up_x;
        r_y= ROWS - right_up_y - 1;
        right_up_x = 0;
        // cout << 5 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_x >= COLS - 2 * radius && right_up_x < COLS && right_up_y >= ROWS - 2 * radius && right_up_y < ROWS){
        r_x = COLS - right_up_x - 1;
        r_y = ROWS - right_up_y - 1;
        // cout << 6 << endl;
    }
    else if(right_up_x >= COLS || right_up_y >= ROWS){
        r_x = COLS - right_up_x - 1;
        r_y = ROWS - right_up_y - 1;
        // cout << 7 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_x < COLS - 2 * radius && right_up_y >= ROWS - 2 * radius && right_up_y < ROWS){
        r_y = ROWS - right_up_y - 1;
        // cout << 8 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_x < COLS - 2 * radius && right_up_y >= ROWS){
        r_y = ROWS - right_up_y - 1;
        // cout << 9 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_y > ROWS - 2 * radius && right_up_x >= COLS - 2 * radius && right_up_x < COLS){
        r_x = COLS - right_up_x - 1;
        // cout << 10 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_y < ROWS - 2 * radius && right_up_x >= COLS){
        r_x = COLS - right_up_y - 1;
        // cout << 11 << endl;
    }
    else if(right_up_x >= 0 && right_up_y < 0 && right_up_x < COLS - 2 * radius && right_up_y >= -2 * radius){
        r_y = 2 * radius + right_up_y;
        right_up_y = 0;
        // cout << 12 << endl;
    }
    else if(right_up_x >= 0 && right_up_y < 0 && right_up_x >= COLS - 2 * radius && right_up_x < COLS && right_up_y >= -2 * radius){
        r_x = COLS - right_up_x;
        r_y = 2 * radius + right_up_y;
        right_up_y = 0;
        // cout << 13 << endl;
    }
    else if(right_up_x >= 0 && right_up_y < 0 && (right_up_x > COLS || right_up_y < -2 * radius)){
        r_x = COLS- right_up_x;
        r_y =2 * radius + right_up_y;
        // cout << 14 << endl;
    }
    // cout << right_up_x << "   " << right_up_y << "   " << r_x << "   " << r_y << endl;
    if(r_x <= 0 || r_y <= 0)
        r_x = 0, r_y = 0;
    else
    {
        if(right_up_x + r_x >= COLS)
        r_x = COLS - right_up_x - 1;
        if(right_up_y + r_y > ROWS)
        r_y = ROWS - right_up_y - 1;
    }
    return cv::Rect2f(right_up_x, right_up_y, r_x, r_y);
}

bool DepthCamera::CenterDistance(const cv::Rect &r1, const cv::Rect &r2)
{
    cv::Point2d p1(r1.x, r1.y);
    cv::Point2d p2(r2.x, r2.y);
    return (centerDistance > distance2d(p1, p2)) ? true : false;
}

double DepthCamera::distance2d(const cv::Point2d p1, const cv::Point2d p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// 处理函数
void DepthCamera::calc_cb(const ros::TimerEvent&)
{
    if(point_aim_.data)
        position.publish(point_aim);
}   

} // namespace CADC