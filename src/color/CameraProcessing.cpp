//
// Created by johannes on 27.03.19.
//

#include <librealsense2/hpp/rs_device.hpp>
#include <librealsense2/hpp/rs_context.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <syslog.h>
#include "CameraProcessing.hpp"

void CameraProcessing::start() {
    IRunnable::start();
}

void CameraProcessing::run() {
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();

    rs2::device selected_device;
    if(devices.size() == 0) {
        syslog(LOG_ERR, "No device found. Aborting!");
        return;
    }
    syslog(LOG_INFO, "Device found.");

    selected_device = devices.front();

    if(!selected_device.is<rs400::advanced_mode>()) {
        syslog(LOG_ERR, "Incompatible module found. Aborting!");
        return;
    }

    rs400::advanced_mode mode(selected_device);
    if(!mode.is_enabled()) {
        mode.toggle_advanced_mode(true);
    }
    mode.load_json("my.json");

    rs2::decimation_filter dec_filter;
    rs2::threshold_filter thr_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

    std::vector<rs2::filter> filters;
    filters.push_back(dec_filter);
    filters.push_back(thr_filter);
    filters.push_back(spat_filter);
    filters.push_back(temp_filter);


    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 6);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, 6);

    pipe.start(cfg);

    intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

    while(!m_stop) {
        rs2::frameset data = pipe.wait_for_frames();
        rs2::depth_frame depth = data.get_depth_frame();
        rs2::frame color = data.get_color_frame();

        for(auto & filter : filters) {
            filter.process(depth);
        }

        color_queue.enqueue(color);
        depth_queue.enqueue(depth);
    }
}

rs2_intrinsics CameraProcessing::getIntrinsics() const {
    return intrinsics;
}
