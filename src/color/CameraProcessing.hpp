//
// Created by johannes on 27.03.19.
//

#ifndef BABOCAM_CAMERAPROCESSING_HPP
#define BABOCAM_CAMERAPROCESSING_HPP


#include "../interfaces/IRunnable.hpp"
#include <librealsense2/rs.hpp>

class CameraProcessing : public IRunnable {
public:
    CameraProcessing(rs2::frame_queue & color, rs2::frame_queue & depth):color_queue(color),depth_queue(depth) {
    }

    rs2_intrinsics & getDepthIntrinsics() { return depth_intrinsics; }
    rs2_intrinsics & getColorIntrinsics() { return color_intrinsics; }

protected:
    void run() override;
private:
    rs2::frame_queue & color_queue;
    rs2::frame_queue & depth_queue;

    rs2_intrinsics depth_intrinsics{};
    rs2_intrinsics color_intrinsics{};
};


#endif //BABOCAM_CAMERAPROCESSING_HPP
