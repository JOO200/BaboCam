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


BallFinder::BallFinder(rs2_intrinsics & intrinsics,
        rs2::frame_queue &color_queue, rs2::frame_queue &depth_queue,
        Context *p_context, double p_diameter)
        :m_context(p_context)
        ,m_diameter(p_diameter)
        ,color_queue(color_queue)
        ,depth_queue(depth_queue)
        ,m_intrinsics(intrinsics) {

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
            if(checkContour(depth_frame, contour, angle, distance)) {
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

bool BallFinder::checkContour(rs2::depth_frame & depth_frame, std::vector<cv::Point> &vector, float & angle, double & distance) {
    using namespace cv;
    Point2f center;
    float radius;
    minEnclosingCircle(vector, center, radius);
    double diameter = radius*2;

    if(center.y < 250 || center.y > 550) return false; // Zu weit unten / oben.

    distance = depth_frame.get_distance(static_cast<int>(center.x), static_cast<int>(center.y));
    if(distance < 0.5 || distance > 2.5) return false;
    double idealD = m_diameter * m_intrinsics.fx / distance;
    angle = -(center.x-(depth_frame.get_width()*0.5))*M_PI/2;

//    syslog(LOG_DEBUG, "Distance[%f], Diameter[%f -> %f], x[%f], y[%f]", distance, diameter, idealD, center.x, center.y);

    if(diameter*0.75 > idealD || diameter*1.5 < idealD) return false;

    double area = contourArea(vector);
//    syslog(LOG_DEBUG, "Second step[%f < %f]?", area, 0.6*diameter*diameter*M_PI/4);
    if(area < 0.6*diameter*diameter*M_PI/4) return false;

#if 0
    dSmth()
#endif
    //TODO:

    //m_context->ball_pos.x;

    return true;
}


