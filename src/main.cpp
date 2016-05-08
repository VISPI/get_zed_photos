/*
* Intel Cornell Cup
* Author: Neel Kowdley <nkowdley@gmail.com>
* Date:05/07/2016
* File: get_disparity.cpp
* This file was adapted from the ZED Sample Code
*/
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
//opencv includes
#include <opencv2/opencv.hpp>
//ZED Includes
#include <zed/Camera.hpp>

using namespace std;

int main(int argc, char **argv) {
	sl::zed::SENSING_MODE dm_type = sl::zed::RAW;
	sl::zed::Camera* zed;
	zed = new sl::zed::Camera(sl::zed::HD720); //use a live image
	//some error checking
	//init WITH self-calibration (- last parameter to false -)
	sl::zed::ERRCODE err = zed->init(sl::zed::MODE::PERFORMANCE, 0, true, false, false);
	// ERRCODE display
	std::cout << "Error code : " << sl::zed::errcode2str(err) << std::endl;

	// Quit if an error occurred
	if (err != sl::zed::SUCCESS) {
		delete zed;
		return 1;
	}
	//initialize opencv display
	int ConfidenceIdx = 100;
	int width = zed->getImageSize().width;
	int height = zed->getImageSize().height;
	cv::Mat disp(height, width, CV_8UC4);
	cv::Size DisplaySize(720, 404);
	cv::Mat dispDisplay(DisplaySize, CV_8UC4);

	//Jetson only. Execute the calling thread on core 2
	sl::zed::Camera::sticktoCPUCore(2);
	// DisparityMap filtering
	//zed->setDispReliability(reliabilityIdx);
    zed->setConfidenceThreshold(ConfidenceIdx);
	bool res = zed->grab(dm_type);
	if (res) {
		cout << "Something went wrong\n";
		return 1;
	}
	slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DISPARITY)).copyTo(disp);
	cv::resize(disp, dispDisplay, DisplaySize);
	//begin saving the disparity image
	std::string filename = std::string(("ZEDDisparity") + std::string(".png")); //create the filename
	cv::Mat dispSnapshot;
	disp.copyTo(dispSnapshot);
	//cv::imshow("Saving Disparity", dispSnapshot);
	cv::imwrite(filename, dispSnapshot);
	return 0;

}
