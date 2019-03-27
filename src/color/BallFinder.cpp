//
// Created by johannes on 27.03.19.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <librealsense2/hpp/rs_export.hpp>
#include <syslog.h>
#include "BallFinder.hpp"



BallFinder::BallFinder(rs2_intrinsics & intrinsics,
        rs2::frame_queue &color_queue, rs2::frame_queue &depth_queue,
        struct context *p_context, double p_diameter)
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
        for(auto & contour : contours) {
            if(checkContour(depth_frame, contour)) {
                syslog(LOG_INFO, "Possible point found!");
            }
        }
    }
}

bool BallFinder::checkContour(rs2::depth_frame & depth_frame, std::vector<cv::Point> &vector) {
    using namespace cv;
    Point2f center;
    float radius;
    minEnclosingCircle(vector, center, radius);
    double diameter = radius*2;

    double distance = depth_frame.get_distance(static_cast<int>(center.x), static_cast<int>(center.y));
    double idealD = m_diameter * m_intrinsics.fx / distance;
    if(diameter*0.75 < idealD || diameter*1.5 > idealD) return false;

    double area = contourArea(vector);
    if(area < 0.6*diameter*diameter*M_PI/4) return false;
    
    //TODO:

    //m_context->ball_pos.x;

    return true;
}


