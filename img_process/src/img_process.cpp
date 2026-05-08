//
// Created by elsa on 2026/5/7.
//

#include "img_process/img_process.hpp"

using namespace std::chrono_literals;
using namespace cv;
using namespace std;

ImgProcess::ImgProcess() : Node("img_process_node") {
    RCLCPP_INFO(this->get_logger(), "img_process_node started");
    this->declare_parameter("B_low_threshold", 0);
    this->declare_parameter("G_low_threshold", 0);
    this->declare_parameter("R_low_threshold", 0);
    this->declare_parameter("B_high_threshold", 0);
    this->declare_parameter("G_high_threshold", 0);
    this->declare_parameter("R_high_threshold", 0);

    this->get_parameter("B_low_threshold", B_low_threshold_);
    this->get_parameter("G_low_threshold", G_low_threshold_);
    this->get_parameter("R_low_threshold", R_low_threshold_);
    this->get_parameter("B_high_threshold", B_high_threshold_);
    this->get_parameter("G_high_threshold", G_high_threshold_);
    this->get_parameter("R_high_threshold", R_high_threshold_);

    wallColor = {58, 58, 58};
    enemy_num_ = 0;
    last_game_start_ = 0;
    shoot_other_enemy_pose.x = 0;
    shoot_other_enemy_pose.y = 0;
    one_outdoor_pose.x = 0;
    one_outdoor_pose.y = 0;
    pixel_status_map.resize(256, std::vector<int>(128, OBSTACLE));
    color_threshold[STAR] = {115, 115, 100, 120, 204, 255};
    color_threshold[BASE] = {100, 100, 51, 110, 204, 128};
    color_threshold[ENEMY_BASE] = {0, 102, 76, 3, 140, 153};
    color_threshold[PURPLEENTRY] = {142, 128, 60, 148, 153, 255};
    color_threshold[GREENENTRY] = {72, 210, 60, 78, 230, 255};
    color_threshold[SENTRY] = {100, 150, 210, 110, 170, 255};
    color_threshold[ENEMY] = {0, 140, 50, 5, 178, 255};

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        10000,
        std::bind(&ImgProcess::imageCallback, this, std::placeholders::_1)
    );
    //map发布
    mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 100000);
    pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
    pubMapInfo = this->create_publisher<robot_msgs::msg::MapInfoMsgs>("/map_info", 100000);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto period_s = std::chrono::seconds(static_cast<int64_t>(10));
    auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(300));
    start_check_timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms,
                                              std::bind(&ImgProcess::check_callback, this));
    move_check_timer_ = rclcpp::create_timer(this, this->get_clock(), period_s,
                                             std::bind(&ImgProcess::move_check_callback, this));
}

void ImgProcess::move_check_callback() {
    move_check = true;
}

void ImgProcess::check_callback() {
    last_game_start_ = sentry_HP_;
}

void ImgProcess::set_map_info(const cv::Mat &Image, uint8_t type) {
    //寻找目标颜色
    cv::Mat binaryImg;
    cv::Mat hsv_image;
    cv::cvtColor(Image, hsv_image, CV_BGR2HSV);
    cv::inRange(hsv_image, cv::Scalar(color_threshold[type][0], color_threshold[type][1], color_threshold[type][2]),
                cv::Scalar(color_threshold[type][3], color_threshold[type][4], color_threshold[type][5]), binaryImg);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(binaryImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    //分类讨论
    if (contours.size() == 0) {
        //RCLCPP_INFO(this->get_logger(), "no %d target", type);
        if (type == ENEMY_BASE) {
            mapInfo[type].is_exist = false;
            mapInfo[type].is_out_of_center = false;
            cv::inRange(hsv_image, cv::Scalar(160, 64, 128), cv::Scalar(163, 90, 178), binaryImg);
            findContours(binaryImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            vector<vector<Point> > contours_poly(contours.size());
            vector<float> radius(contours.size());
            vector<Point2f> centers(contours.size());
            for (size_t i = 0; i < contours.size(); i++) {
                approxPolyDP(contours[i], contours_poly[i], 3, true);
                minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
            }
            if (contours.size() == 1 && radius[0] > 6) {
                mapInfo[ENEMY_BASE].pos.x = centers[0].x / 40;
                mapInfo[ENEMY_BASE].pos.y = 12.8 - centers[0].y / 40;
            }
        } else if (type == ENEMY) {
            enemy_num_ = 0;
            cv::inRange(hsv_image, cv::Scalar(25, 127, 90), cv::Scalar(35, 200, 200), binaryImg);
            findContours(binaryImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            vector<vector<Point> > contours_poly(contours.size());
            vector<float> radius(contours.size());
            vector<Point2f> centers(contours.size());
            for (size_t i = 0; i < contours.size(); i++) {
                approxPolyDP(contours[i], contours_poly[i], 3, true);
                minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
                if (radius[i] > 3) {
                    enemy_num_ = 1;
                    break;
                }
            }
            if (enemy_num_ == 0) {
                mapInfo[type].is_exist = false;
                mapInfo[type].is_out_of_center = false;
            }
        }
        return;
    }
    // 寻找最小外接圆
    vector<vector<Point> > contours_poly(contours.size());
    vector<float> radius(contours.size());
    vector<Point2f> centers(contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
        approxPolyDP(contours[i], contours_poly[i], 3, true);
        minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
    }
    if (type != SENTRY && type != ENEMY) {
        for (size_t i = 0; i < contours.size(); i++) {
            if (radius[i] > 2) {
                for (int j = -1; j <= 1; j++) {
                    for (int k = -1; k <= 1; k++) {
                        pixel_status_map[int(centers[i].x / 4) + j][128 - int(centers[i].y / 4) + k] = REACHABLE;
                    }
                }
            }
        }
    }
    if (contours.size() == 1) {
        // cv::circle(Image, centers[0], radius[0], cv::Scalar(0, 0, 255), 2);
        switch (type) {
            case STAR:
                if (radius[0] > 6) {
                    mapInfo[type].pos.x = centers[0].x / 40;
                    mapInfo[type].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[type].is_exist = true;
                    mapInfo[type].is_out_of_center = false;
                }
                break;
            case BASE:
            case ENEMY_BASE:
                if (radius[0] > 6) {
                    mapInfo[type].pos.x = centers[0].x / 40;
                    mapInfo[type].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[type].is_exist = true;
                    mapInfo[type].is_out_of_center = true;
                }
                break;
            case SENTRY:
            case GREENENTRY:
            case PURPLEENTRY:
                if (radius[0] > 6) {
                    mapInfo[type].pos.x = centers[0].x / 40;
                    mapInfo[type].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[type].is_exist = true;
                    mapInfo[type].is_out_of_center = isOutOfRange(centers[0]);
                    if (onOutDoor(centers[0])) {
                        one_outdoor_pose = cv::Point2f(1000, 1000);
                    }
                } else if (type == PURPLEENTRY || type == GREENENTRY) {
                    if (!isInDoor(centers[0]) && !isFarFromSentry(centers[0])) {
                        one_outdoor_pose = centers[0];
                    }
                }
                break;
            case ENEMY:
                mapInfo[type].pos.x = centers[0].x / 40;
                mapInfo[type].pos.y = 12.8 - centers[0].y / 40;
                if (isFarFromEnemyBase(centers[0])) {
                    mapInfo[type].is_exist = true;
                    mapInfo[type].is_out_of_center = isOutOfRange(centers[0]);
                    enemy_num_ = 1;
                } else {
                    mapInfo[type].is_exist = true;
                    mapInfo[type].is_out_of_center = false;
                    enemy_num_ = 0;
                }
                //RCLCPP_WARN(this->get_logger(), "enemy_num_:%d",enemy_num_);
                break;
            default:
                break;
        }
    } else {
        switch (type) {
            case STAR:
            case BASE:
            case ENEMY_BASE:
            case SENTRY:

                // RCLCPP_WARN(this->get_logger(), "more than one object detected,type:%d",type);
                break;
            case GREENENTRY:
            case PURPLEENTRY:
                for (size_t i = 0; i < contours.size(); i++) {
                    if (radius[i] > 6) {
                        mapInfo[type].pos.x = centers[i].x / 40;
                        mapInfo[type].pos.y = 12.8 - centers[i].y / 40;
                        mapInfo[type].is_exist = true;
                        mapInfo[type].is_out_of_center = isOutOfRange(centers[i]);
                        // cv::circle(Image, centers[i], radius[i], cv::Scalar(0, 0, 255), 2);
                        if (Image.at<cv::Vec3b>(centers[i].y, centers[i].x) == cv::Vec3b(240, 170, 89)) {
                            RCLCPP_INFO(this->get_logger(), "Transfering");
                        }
                        if (onOutDoor(centers[i])) {
                            one_outdoor_pose = cv::Point2f(1000, 1000);
                        }
                    } else if (!isInDoor(centers[i]) && !isFarFromSentry(centers[i])) {
                        one_outdoor_pose = centers[i];
                    }
                }
                break;
            case ENEMY: {
                double min_dist = 10000.0;
                enemy_num_ = 0;
                for (size_t i = 0; i < contours.size(); i++) {
                    // cv::circle(Image, centers[i], radius[i], cv::Scalar(0, 0, 255), 2);
                    if (radius[i] > 5 &&
                        isFarFromEnemyBase(centers[i])) {
                        enemy_num_ += 1;
                        if (enemy_num_ == 1) {
                            shoot_other_enemy_pose = centers[i];
                        }
                        if (distance(centers[i], mapInfo[SENTRY].pos) - isOutOfRange(centers[i]) * 1000 < min_dist) {
                            mapInfo[type].pos.x = centers[i].x / 40;
                            mapInfo[type].pos.y = 12.8 - centers[i].y / 40;
                            mapInfo[type].is_exist = true;
                            mapInfo[type].is_out_of_center = isOutOfRange(centers[i]);
                            min_dist = distance(centers[i], mapInfo[SENTRY].pos) - mapInfo[type].is_out_of_center *
                                       1000;
                        } else {
                            shoot_other_enemy_pose = centers[i];
                        }
                    }
                }
            }
            break;
            default:
                break;
        }
    }
}

void ImgProcess::imageCallback(sensor_msgs::msg::Image rosImage) {
    this->get_parameter("B_low_threshold", B_low_threshold_);
    this->get_parameter("G_low_threshold", G_low_threshold_);
    this->get_parameter("R_low_threshold", R_low_threshold_);
    this->get_parameter("B_high_threshold", B_high_threshold_);
    this->get_parameter("G_high_threshold", G_high_threshold_);
    this->get_parameter("R_high_threshold", R_high_threshold_);

    auto cvImage = cv_bridge::toCvCopy(rosImage, rosImage.encoding);
    cv::Mat img = cvImage->image;
    cvtColor(img, img, CV_RGB2BGR);

    //HPupdate
    int cout = 0;
    for (int i = 0; i <= 380; ++i) {
        int x = i;
        int y = 996;
        cv::Vec3b pixel = img.at<cv::Vec3b>(cv::Point(x, y));
        //RCLCPP_INFO(this->get_logger(),"R:%d,G:%d,B:%d",pixel[0],pixel[1],pixel[2]);
        if (pixel[0] == 131 && pixel[1] == 131 && pixel[2] == 131) {
            cout++;
        } else if (pixel[0] == 106 && pixel[1] == 106 && pixel[2] == 106) {
            cout++;
        }
    }
    sentry_HP_ = cout / 3.8;
    RCLCPP_ERROR(this->get_logger(), "HP: %f", sentry_HP_);

    // if (sentry_HP_==0){//游戏重开
    //     //    RCLCPP_INFO(this->get_logger(),"game over");
    //     for (auto &row : pixel_status_map){
    //         std::fill(row.begin(), row.end(), OBSTACLE);
    //     }
    //     for (int i = 0; i < 8; ++i) {
    //         mapInfo[i].is_exist_and_out_range=false;
    //         mapInfo[i].pos.x=1000;
    //         mapInfo[i].pos.y=1000;
    //     }
    //     one_outdoor_pose.x=1000;
    //     one_outdoor_pose.y=1000;
    // }

    //bullet update
    cout = 0;
    for (int i = 2032; i <= 2047; ++i) {
        int x = i;
        int y = 1014;
        cv::Vec3b pixel = img.at<cv::Vec3b>(cv::Point(x, y));
        if (pixel[0] >= 200 && pixel[1] >= 200 && pixel[2] >= 200) {
            cout++;
        }
        y = 1013;
        pixel = img.at<cv::Vec3b>(cv::Point(x, y));
        if (pixel[0] >= 200 && pixel[1] >= 200 && pixel[2] >= 200) {
            cout++;
        }
    }
    if (cout == 0) {
        is_bullet_low_ = true;
    } else {
        is_bullet_low_ = false;
    }
    RCLCPP_ERROR(this->get_logger(), "is_bullet_low?: %d", is_bullet_low_);

    //地图处理
    int newWidth = 256; // 新的宽度
    int newHeight = 128; // 新的高度
    cv::Mat mapImage, findingImage, viewImage;
    // 使用cv::resize函数降低分辨率
    cv::resize(img, mapImage, cv::Size(newWidth, newHeight), 0, 0, cv::INTER_NEAREST);
    cv::resize(img, findingImage, cv::Size(newWidth * 4, newHeight * 4), 0, 0, cv::INTER_NEAREST);
    for (size_t i = 0; i < 7; i++) {
        set_map_info(findingImage, i);
    }

    double distance1 = sqrt(pow(mapInfo[SENTRY].pos.x - mapInfo[PURPLEENTRY].pos.x, 2)
                            + pow(mapInfo[SENTRY].pos.y - mapInfo[PURPLEENTRY].pos.y, 2));
    double distance2 = sqrt(pow(mapInfo[SENTRY].pos.x - mapInfo[GREENENTRY].pos.x, 2)
                            + pow(mapInfo[SENTRY].pos.y - mapInfo[GREENENTRY].pos.y, 2));
    is_transfering_ = (distance1 <= 0.3 || distance2 <= 0.3);
    RCLCPP_INFO(get_logger(), "is_transfering?: %d", is_transfering_);

    publish_map(mapImage, wallColor);


    // cv::Mat rrImg;
    // cv::resize(mapImage, rrImg,cv::Size(256*5, 128*5),0,0, cv::INTER_LINEAR);
    // cv::imshow("v", rrImg);
    // cv::waitKey(1);

    // RCLCPP_INFO(this->get_logger(), "----------------sentrypos: %f,%f",mapInfo[SENTRY].pos.x,mapInfo[SENTRY].pos.y);

    //test
    // cv::Mat testImage;
    // cv::inRange(findingImage, cv::Scalar(B_low_threshold_, G_low_threshold_, R_low_threshold_), cv::Scalar(B_high_threshold_, G_high_threshold_, R_high_threshold_), testImage);
    // vector<vector<Point> > test_contours;
    // vector<Vec4i> test_hierarchy;
    // findContours( testImage, test_contours, test_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    // vector<vector<Point> > test_contours_poly( test_contours.size() );
    // vector<float>test_radius( test_contours.size() );
    // vector<Point2f>test_centers( test_contours.size() );
    // for( size_t i = 0; i < test_contours.size(); i++ )
    // {
    //     approxPolyDP( test_contours[i], test_contours_poly[i], 3, true );
    //     minEnclosingCircle( test_contours_poly[i], test_centers[i], test_radius[i] );
    //     if(test_radius[i]>4){
    // cv::circle( findingImage,cv::Point2f(40*mapInfo[ENEMY_BASE].pos.x,-40*(mapInfo[ENEMY_BASE].pos.y-12.8)),5,cv::Scalar( 0, 0, 255 ),10);
    // }
    // }
    // 绘制 ENEMY_BASE 区域的框
    // cv::rectangle(img, cv::Point(1900,1000), cv::Point(2048,1024), cv::Scalar(0, 0, 255), 2); // 红色框
    // cv::rectangle(img, cv::Point(0,975), cv::Point(390,1024), cv::Scalar(0, 0, 255), 2); // 红色框
    // // // cv::putText(img, "HP: " + std::to_string(sentry_HP_),cv::Point(10, 950), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

    for (size_t i = 0; i < 8; i++) {
        cv::circle(findingImage, cv::Point2f(40 * mapInfo[i].pos.x, -40 * (mapInfo[i].pos.y - 12.8)), 6,
                   cv::Scalar(30 * i, 20 * i, 255 - 30 * i), 2);
    }
    cv::circle(findingImage, one_outdoor_pose, 6, cv::Scalar(255, 255, 255), 2);
    cv::imshow("view", findingImage);
    waitKey(1);

    // if(game_mode_==HARD&&enemy_num_>0&&mapInfo[BASE].is_exist_and_out_range==false){//敌人在范围内，且地图上没有base，则远离
    //     std::vector<cv::Point> points = getExtendedPoints(cv::Point(int(mapInfo[ENEMY].pos.x*10),int(mapInfo[ENEMY].pos.y*10)), cv::Point(int(mapInfo[SENTRY].pos.x*10),int(mapInfo[SENTRY].pos.y*10)));
    //     for (const cv::Point& p : points) {
    //         if (pixel_status_map[p.x][p.y] != OBSTACLE) {
    //             mapInfo[UNEXPLOREDPOSE].pos.x=p.x/10.0;
    //             mapInfo[UNEXPLOREDPOSE].pos.y=p.y/10.0;
    //         }
    //     }
    // }
    publish_sentry_odom(pubOdomAftMapped, tf_broadcaster);
    publish_map_info();
}

void ImgProcess::publish_map(const cv::Mat resizedImage, const cv::Vec3b wallColor) {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = this->get_clock()->now();
    map.info.resolution = 0.1; // float32
    map.info.width = 256; // uint32
    map.info.height = 128; // uint32
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(map.info.width * map.info.height);
    for (int i = 0; i < resizedImage.rows; ++i) {
        for (int j = 0; j < resizedImage.cols; ++j) {
            cv::Vec3b pixel = resizedImage.at<cv::Vec3b>(cv::Point(j, resizedImage.rows - i));
            int index = i * resizedImage.cols + j;

            if (pixel[0] == wallColor[0] && pixel[1] == wallColor[1] && pixel[2] == wallColor[2]) {
                map.data[index] = 100;
            } else {
                map.data[index] = 0;
            }

            if (sentry_HP_ != 0 && (pixel[0] + pixel[1] + pixel[2]) > 100) {
                pixel_status_map[j][i] = REACHABLE;
                map.data[index] = REACHABLE;
            } else {
                map.data[index] = pixel_status_map[j][i];
            }
            if (i == 0 || j == 0 || i == resizedImage.rows - 1 || j == resizedImage.cols - 1 ||
                (j >= 237 && i >= 0 && j <= 256 && i <= 3) || //子弹区
                (j >= 0 && i >= 0 && j <= 49 && i <= 7) || //血量区
                (j >= 0 && i >= 126 && j <= 3 && i <= 128) || //帧率区
                (last_game_start_ == 0 && i >= 0 && i <= 128 && j >= 0 && j <= 256)
            ) {
                map.data[index] = OBSTACLE;
                pixel_status_map[j][i] = OBSTACLE;
            }
        }
    }

    // cv::Mat rImg;
    // cv::resize(resizedImage, rImg,cv::Size(256*5, 128*5),0,0, cv::INTER_LINEAR);
    // cv::imshow("map", rImg);
    // cv::waitKey(1);
    mapPublisher->publish(map);
    // RCLCPP_INFO(this->get_logger(), "map published");
}

void ImgProcess::publish_sentry_odom(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
                                     std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br) {
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = robot_frame;
    odomAftMapped.header.stamp = this->get_clock()->now();
    set_posestamp(odomAftMapped.pose.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transformStamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transformStamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transformStamped.header.stamp = rclcpp::Time(odomAftMapped.header.stamp);
    transformStamped.header.frame_id = map_frame;
    transformStamped.child_frame_id = robot_frame;
    // std::cout << "a"<<std::endl;
    tf_br->sendTransform(transformStamped);
}

void ImgProcess::publish_map_info() {
    robot_msgs::msg::MapInfoMsgs mapInfoMsgs;
    if (!move_check) {
        std::vector<robot_msgs::msg::MapInfo> temp;
        robot_msgs::msg::MapInfo tp;
        temp.reserve(7);
        for (int i = 0; i < 7; i++) {
            // tp.pos = mapInfo[UNEXPLOREDPOSE].pos;
            tp.is_exist = mapInfo[i].is_exist;
            tp.is_out_of_center = mapInfo[i].is_out_of_center;
            temp.push_back(tp);
        }
        mapInfoMsgs.map_info = temp;
    } else {
        mapInfoMsgs.map_info = mapInfo;
    }
    mapInfoMsgs.enemy_num = enemy_num_;
    mapInfoMsgs.sentry_hp = sentry_HP_;
    mapInfoMsgs.is_transfering = is_transfering_;
    mapInfoMsgs.is_bullet_low = is_bullet_low_;
    pubMapInfo->publish(mapInfoMsgs);
}

template<typename T>
void ImgProcess::set_posestamp(T &out) {
    out.position.x = mapInfo[SENTRY].pos.x;
    out.position.y = mapInfo[SENTRY].pos.y;
    out.position.z = 0.0;
    out.orientation.x = 0.0;
    out.orientation.y = 0.0;
    out.orientation.z = 0.0;
    out.orientation.w = 1.0;
}
