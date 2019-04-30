//
// Created by johannes on 27.03.19.
//

#ifndef BABOCAM_BALLFINDER_HPP
#define BABOCAM_BALLFINDER_HPP


#include <librealsense2/hpp/rs_processing.hpp>
#include "../interfaces/IRunnable.hpp"
#include "../struct/Context.hpp"
#include "CameraProcessing.hpp"

class BallFinder : public IRunnable {
public:
    BallFinder(CameraProcessing * processing,
               rs2::frame_queue &color_queue, rs2::frame_queue &depth_queue,
               Context *p_context, double p_diameter);

protected:
    void run() override;
private:
    rs2::frame_queue & color_queue;
    rs2::frame_queue & depth_queue;
    double m_diameter;
    Context* m_context;
    CameraProcessing * processing;

    cv::Mat ball_template;

    bool checkContour(rs2::depth_frame & depth_frame, rs2::frame & color, std::vector<cv::Point> &vector, float & angle, double & distance);
};


#endif //BABOCAM_BALLFINDER_HPP
