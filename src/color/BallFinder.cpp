//
// Created by johannes on 27.03.19.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <librealsense2/hpp/rs_export.hpp>
#include <syslog.h>
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
    while(!m_stop) {
        frame color_frame = color_queue.wait_for_frame();
        depth_frame depth_frame = depth_queue.wait_for_frame().as<rs2::depth_frame>();

        const int w = depth_frame.get_width();
        const int h = depth_frame.get_height();

        frame colorized_depth = color_map.process(depth_frame);
        Mat depth_mat = Mat(Size(w, h), CV_16UC1, (void*)colorized_depth.get_data(), Mat::AUTO_STEP);

        std::vector<std::vector<Point>> contours;
        findContours(depth_mat, contours, RETR_LIST, CHAIN_APPROX_NONE);
        std::vector<Navigator2D> ballPositions;
        for(auto & contour : contours) {
            float angle;
            double distance;
            if(checkContour(depth_frame, contour, angle, distance)) {
                syslog(LOG_INFO, "Possible point found!");
                ballPositions.emplace_back(distance, angle);
            }
        }
        if(ballPositions.size() == 1) {
            m_context->setBall(ballPositions.front());
        } else {
            syslog(LOG_INFO, "No possible ball found.");
        }
    }
}

bool BallFinder::checkContour(rs2::depth_frame & depth_frame, std::vector<cv::Point> &vector, float & angle, double & distance) {
    using namespace cv;
    Point2f center;
    float radius;
    minEnclosingCircle(vector, center, radius);
    double diameter = radius*2;

    angle = (center.x-depth_frame.get_width())*M_PI/2;

    distance = depth_frame.get_distance(static_cast<int>(center.x), static_cast<int>(center.y));
    double idealD = m_diameter * m_intrinsics.fx / distance;
    if(diameter*0.75 < idealD || diameter*1.5 > idealD) return false;

    double area = contourArea(vector);
    if(area < 0.6*diameter*diameter*M_PI/4) return false;

#if 0
    dSmth()
#endif
    //TODO:

    //m_context->ball_pos.x;

    return true;
}


