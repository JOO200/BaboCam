/*
 * DepthStreamThread.hpp
 *
 *  Created on: 18.02.2019
 *      Author: johannes
 */

#ifndef SRC_BABOCAM_COLOR_COLORSTREAMTHREAD_HPP_
#define SRC_BABOCAM_COLOR_COLORSTREAMTHREAD_HPP_

#include <opencv2/opencv.hpp>
#include <thread>
#include <librealsense2/rs.hpp>

class ColorStream {
public:
	ColorStream();
	~ColorStream();
	void init(rs2::pipeline * pipeline);
	void start();
	void stop();
private:
	void run();
	cv::Mat img;
	rs2::colorizer color_map;
	rs2::pipeline * pipeline;


	// Thread stuff:
	std::thread* m_thread;
	bool stop_thread;
};



#endif /* SRC_BABOCAM_COLOR_COLORSTREAMTHREAD_HPP_ */
