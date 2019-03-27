//
// Created by johannes on 27.03.19.
//

#ifndef BABOCAM_CAMERAPROCESSING_HPP
#define BABOCAM_CAMERAPROCESSING_HPP


#include "../interfaces/IRunnable.hpp"

class CameraProcessing : public IRunnable {
public:
    CameraProcessing(rs2::frame_queue & color, rs2::frame_queue & depth):color_queue(color),depth_queue(depth) {

    }

    void start() override;

    rs2_intrinsics getIntrinsics() const;

protected:
    void run() override;
private:
    rs2::frame_queue & color_queue;
    rs2::frame_queue & depth_queue;

    rs2_intrinsics intrinsics{};
};


#endif //BABOCAM_CAMERAPROCESSING_HPP
