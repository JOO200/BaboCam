/*
 * DepthStreamThread.cpp
 *
 *  Created on: 18.02.2019
 *      Author: johannes
 */

#include "ColorStreamThread.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include <syslog.h>
#include <ctime>


ColorStream::ColorStream() {
	m_thread = NULL;
	pipeline = NULL;
	stop_thread = false;
}

ColorStream::~ColorStream() {

}

void ColorStream::init(rs2::pipeline * p_pipeline) {
	pipeline = p_pipeline;
}

void ColorStream::start() {
	m_thread = new std::thread(&ColorStream::run, this);
}

void ColorStream::stop() {
	//if(m_thread == NULL) return;
	//stop_thread = true;
	m_thread->join();
	m_thread = NULL;
}

void ColorStream::run() {
	try {
		const auto window_name = "Display Image";
		syslog(LOG_INFO, "Starting ColorStream.");
		color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2);

		using namespace cv;
		namedWindow(window_name);
		Mat gray;
		while(!stop_thread && cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) > 0) {
			clock_t begin = clock();
			rs2::frameset data = pipeline->wait_for_frames();
			rs2::frame video = data.get_color_frame().apply_filter(color_map);
			syslog(LOG_INFO, "Elapsed time %u", clock()-begin);
			const int w = video.as<rs2::video_frame>().get_width();
			const int h = video.as<rs2::video_frame>().get_height();
/*
			rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
			const int w_depth = depth.as<rs2::video_frame>().get_width();
			const int h_depth = depth.as<rs2::video_frame>().get_height();
*/
			img = Mat(Size(w, h), CV_8UC3, (void*)video.get_data(), Mat::AUTO_STEP);
			cvtColor(img, img, COLOR_BGR2RGB);
	/*		cvtColor(img, gray, COLOR_BGR2GRAY);

			GaussianBlur(gray, gray, Size(9, 9), 2, 2);
			std::vector<Vec3f> circles;
			HoughCircles(gray, circles, HOUGH_GRADIENT, 1, gray.rows/8, 100, 80, 0, 0);
			for(size_t i = 0; i < circles.size(); i++) {
				syslog(LOG_INFO, "Circle found! x=%f, y=%f, r=%f", circles[i][0], circles[i][1], circles[i][2]);
		}*/
			imshow(window_name, img);
		}
	}
	catch(rs2::error& e) {
		syslog(LOG_ERR, "Failed message %s", e.what());
		syslog(LOG_ERR, "Failed function %s", e.get_failed_function().c_str());
		syslog(LOG_ERR, "Failed args %s", e.get_failed_args().c_str());
	}
	syslog(LOG_INFO, "Stop ColorStream.");
}
