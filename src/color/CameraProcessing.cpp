//
// Created by johannes on 27.03.19.
//

#include <librealsense2/hpp/rs_device.hpp>
#include <librealsense2/hpp/rs_context.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <syslog.h>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include "CameraProcessing.hpp"

void CameraProcessing::run() {
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();

    rs2::device selected_device;
    if(devices.size() == 0) {
        syslog(LOG_ERR, "No device found. Aborting!");
        return;
    }
    syslog(LOG_INFO, "Device found.");
    selected_device = devices[0];

    if (selected_device.is<rs400::advanced_mode>())
    {
        auto advanced_mode_dev = selected_device.as<rs400::advanced_mode>();
        // Check if advanced-mode is enabled
        if (!advanced_mode_dev.is_enabled())
        {
            // Enable advanced-mode
            advanced_mode_dev.toggle_advanced_mode(true);
        }
        // Select the custom configuration file
        std::ifstream t("/home/johannes/dev/BaboCam/my.json");
        std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
        syslog(LOG_INFO, "JSON: %s", preset_json.c_str());
        advanced_mode_dev.load_json(preset_json);
        syslog(LOG_INFO, "Config loaded.");

    }
    else
    {
        syslog(LOG_ERR, "Current device doesn't support advanced-mode!");
        return;
    }

    rs2::decimation_filter dec_filter;
    rs2::threshold_filter thr_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

    std::vector<rs2::filter> filters;
    filters.push_back(dec_filter);
    filters.push_back(thr_filter);
    filters.push_back(spat_filter);
    filters.push_back(temp_filter);


    rs2::pipeline pipe = rs2::pipeline(ctx);
    rs2::config cfg;
    syslog(LOG_INFO, "Enable config.");

    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 6);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 6);

    pipe.start(cfg);
    syslog(LOG_INFO, "Pipe started.");

    depth_intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    color_intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();

    while(!m_stop) {
        rs2::frameset data = pipe.wait_for_frames();
        rs2::depth_frame depth = data.get_depth_frame();
        rs2::frame color = data.get_color_frame();
#define USE_FILTERS
#ifdef USE_FILTERS
        for(auto & filter : filters) {
            filter.process(depth);
        }
#endif
        color_queue.enqueue(color);
        depth_queue.enqueue(depth);
    }
}
