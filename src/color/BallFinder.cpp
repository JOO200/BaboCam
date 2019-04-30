//
// Created by johannes on 27.03.19.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <librealsense2/hpp/rs_export.hpp>
#include <syslog.h>
#include <opencv2/opencv.hpp>
#include "BallFinder.hpp"
#include "../math/Navigator2D.hpp"
#include "../struct/Context.hpp"


BallFinder::BallFinder(CameraProcessing * processing,
        rs2::frame_queue &color_queue, rs2::frame_queue &depth_queue,
        Context *p_context, double p_diameter)
        :m_context(p_context)
        ,m_diameter(p_diameter)
        ,color_queue(color_queue)
        ,depth_queue(depth_queue)
        ,processing(processing) {
    ball_template = cv::imread("/home/johannes/dev/BaboCam/fuenfeck.png");
}

void BallFinder::run() {
    using namespace cv;
    using namespace rs2;
    colorizer color_map;
    namedWindow("test", WINDOW_AUTOSIZE);
    while(!m_stop) {
        frame color_frame = color_queue.wait_for_frame();
        depth_frame depth_frame = depth_queue.wait_for_frame().as<rs2::depth_frame>();

        const int w = depth_frame.get_width();
        const int h = depth_frame.get_height();

        frame colorized_depth = color_map.process(depth_frame);
        Mat mat = Mat(Size(w, h), CV_8UC3, (void*)colorized_depth.get_data(), Mat::AUTO_STEP);
        Mat depth_mat;
        cvtColor(mat, depth_mat, COLOR_BGR2GRAY);

        std::vector<std::vector<Point>> contours;
        findContours(depth_mat, contours, RETR_LIST, CHAIN_APPROX_NONE);
        std::vector<Navigator2D> ballPositions;
        for(auto & contour : contours) {
            float angle;
            double distance;
            if(checkContour(depth_frame, color_frame, contour, angle, distance)) {
                ballPositions.emplace_back(distance, angle);
            }
        }
        if(ballPositions.size() == 1) {
            m_context->setBall(ballPositions.front());
            m_context->getCond().notify_all();
            syslog(LOG_INFO, "Ball found.");
        } else if(ballPositions.size() > 1){
            syslog(LOG_INFO, "multiple possible ball found. %zu", ballPositions.size());
        } else {
            syslog(LOG_INFO, "No possible ball found.");
        }
        imshow("test", mat);
        cv::waitKey(1);
    }
}

bool BallFinder::checkContour(rs2::depth_frame & depth_frame, rs2::frame & color, std::vector<cv::Point> &vector, float & angle, double & distance) {
    using namespace cv;
    Point2f center;
    float radius;
    minEnclosingCircle(vector, center, radius);
    double diameter = radius*2;

    if(center.y < 250 || center.y > 550) return false; // Zu weit unten / oben.

    distance = depth_frame.get_distance(static_cast<int>(center.x), static_cast<int>(center.y));
    if(distance < 0.2 || distance > 2.5) return false;
    double idealD = m_diameter * processing->getDepthIntrinsics().fx / distance;
//    syslog(LOG_DEBUG, "Distance[%f], Diameter[%f -> %f], x[%f], y[%f]", distance, diameter, idealD, center.x, center.y);

    float dx = distance * (center.x - processing->getDepthIntrinsics().ppx) / processing->getDepthIntrinsics().fx;
    angle = std::asin(dx/distance);

    if(diameter*0.85 > idealD || diameter*1.5 < idealD) return false;

    double area = contourArea(vector);
//    syslog(LOG_DEBUG, "Second step[%f < %f]?", area, 0.6*diameter*diameter*M_PI/4);
    if(area < 0.6*diameter*diameter*M_PI/4) return false;
    syslog(LOG_INFO, "distance %f, dx %f, angle %f", distance, dx, angle);
#ifdef USE_COLOR
    float dy = distance * (center.y - processing->getDepthIntrinsics().ppy) / processing->getDepthIntrinsics().fy;
    if(dx < 0 || dy < 0) return false;
    if(dx > 1280 || dy > 720) return false;
    //angle = -(center.x-(depth_frame.get_width()*0.5))*M_PI/2;

    float color_x = (processing->getColorIntrinsics().fx * dx / distance) + processing->getColorIntrinsics().ppx;
    float color_y = (processing->getColorIntrinsics().fy * dy / distance) + processing->getColorIntrinsics().ppy;
    int min_x = std::max(color_x-100.0, 0.0), max_x = std::min(color_x+100.0, 1280.0);
    int min_y = std::max(color_y-100.0, 0.0), max_y = std::min(color_y+100.0, 1280.0);


    cv::Mat color_img = cv::Mat(Size(1280, 720), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

    cv::Mat shrinked = cv::Mat(color_img, Range(min_y, max_y), Range(min_x, max_x));

    cv::Mat matched;
    cv::matchTemplate(shrinked, ball_template, matched, cv::TM_SQDIFF);
    double minLoc;
    Point position;
    cv::minMaxLoc(matched, &minLoc, nullptr, &position);
    std::cout << "MinX: " << min_x << " MaxX: " << max_x << " MinY: " << min_y << " MaxY: " << max_y << " MinLoc: " << minLoc << std::endl;

    circle(color_img, position, 5, Scalar(255,0,0), 2);
    imshow("Test", shrinked);

    if(std::abs(position.x - color_x) > 200 || std::abs(position.y - color_y) > 200) {
        return false;
    }
#endif

    return true;
}


