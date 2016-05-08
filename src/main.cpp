/*
* Intel Cornell Cup
* Author: Neel Kowdley <nkowdley@gmail.com>
* Date:05/07/2016
* File: get_disparity.cpp
* This file was adapted from the ZED Sample Code
*/
//standard libraries
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
//opencv includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//ZED Includes
#include <zed/Camera.hpp>
//Local includes
#include "SimpleGPIO.h"

//define the max iterations
#define MAX_TIME 1000

using namespace std;
using namespace cv;

Mat src; Mat src_gray; Mat src_gray_cp; Mat opIm; Mat element; Mat threshold_output; Mat overlay;

//Threshold Characteristics
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

//Opening Characteristics
int morph_elem = 0;
int morph_size = 5;
int operation = 2;
bool* _x1;
bool* _y1;
bool* _x2;
bool* _y2;
bool* _dp;
//int const max_elem = 2;
//int const max_kernel_size = 21;
const char* window_name = "Opening";

//Function Declarations
void thresh_contour(int, void*);
void opening(int, void*);
void intToBin(bool*, int);
void save_img(sl::zed::Camera* zed, cv::Mat disp);

//GPIO test
const int GPIO_LED = 57;

int main(int argc, char **argv) {
	//GPIO test
	gpio_export(GPIO_LED);
	gpio_set_dir(GPIO_LED, OUTPUT_PIN);
	//initialize a counter and a timer
	int count = 0;
	double t1 = (double)getTickCount();
	//zed Initialization
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
	while(count < MAX_TIME) {
		if (res) {
			cout << "Something went wrong\n";
			return 1;
		}
		slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DISPARITY)).copyTo(disp);
		cv::resize(disp, dispDisplay, DisplaySize);
		//begin saving the disparity image
		//Don't write the image anymore, uncomment this to write out the file
		//save_img(camera,disp,count)

		//Coordinate Initialization
		_x1 = (bool*) calloc (10,sizeof(bool));
		_y1 = (bool*) calloc (10,sizeof(bool));
		_x2 = (bool*) calloc (10,sizeof(bool));
		_y2 = (bool*) calloc (10,sizeof(bool));
		_dp = (bool*) calloc (10,sizeof(bool));

		cvtColor( disp, src_gray, CV_BGR2GRAY );

		src_gray_cp= src_gray.clone();

		const char* source_window = "Source";
		namedWindow( source_window, CV_WINDOW_AUTOSIZE);
		imshow( source_window, src);

		const char* blur_window = "Blur";

		//createTrackbar (" Threshold:", "Source", &thresh, max_thresh, thresh_contour ); //Trackbar for testing threshold setting
		//createTrackbar( "Element:\n 0:Rect | 1:Cross | 2:Ellipse","Opening",&morph_elem, max_elem, opening);
		//createTrackbar( "Kernel size:\n 2n+1", "Opening", &morph_size, max_kernel_size, opening );
		opening(0,0);
		thresh_contour(0,0);
		res = zed->grab(dm_type);
		count++;
	}
	double t2 = (double)getTickCount();

	double time = (t2 - t1)/getTickFrequency();
	cout << "\nTotal Runtime is: " << time << "\n";
	return 0;
}

void save_img(sl::zed::Camera* zed, cv::Mat disp, int count) {
	std::string filename = std::string(("ZEDDisparity") + count + std::string(".png")); //create the filename
	cv::Mat dispSnapshot;
	disp.copyTo(dispSnapshot);
	//Don't write the image anymore, uncomment this to write out the file
	cv::imwrite(filename, dispSnapshot);
}
/*Begin Helper Functions*/
void thresh_contour(int, void*)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	//Detect Edges Using Threshold
	threshold( opIm, threshold_output, thresh, 255, THRESH_BINARY );
	blur(threshold_output, threshold_output, Size(10,10));
	//Find Contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

	//Approximate contours to polygons and get bouding rectangles
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect ( contours.size() );

	for( int ii = 0; ii < contours.size(); ii++)
	{
		approxPolyDP( Mat (contours[ii]), contours_poly[ii], 3, true);
		boundRect[ii] = boundingRect( Mat(contours_poly[ii]) );
	}

	Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3);

	//Coordinate Array Initializations
	int xtl [(contours.size()-1)];
	int ytl [(contours.size()-1)];
	int xbr [(contours.size()-1)];
	int ybr [(contours.size()-1)];
	int xc  [(contours.size()-1)];
	int yc  [(contours.size()-1)];
	int depth [(contours.size()-1)];

	for( int ii = 0; ii < contours.size(); ii++)
	{
		Scalar color = Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours_poly, ii, color, 1, 8, vector<Vec4i>(), 0, Point() );
		rectangle( drawing, boundRect[ii].tl(), boundRect[ii].br(), color, 2, 8, 0);
		rectangle(src_gray, boundRect[ii].tl(), boundRect[ii].br(), color, 2,8,0);

		//Saving Coordinates in Array
		xtl[ii] = boundRect[ii].x;
		ytl[ii] = boundRect[ii].y;
		xbr[ii] = boundRect[ii].x + boundRect[ii].width;
		ybr[ii] = boundRect[ii].y + boundRect[ii].height;
		xc[ii]  = boundRect[ii].x + (boundRect[ii].width >> 1);
		yc[ii]  = boundRect[ii].y + (boundRect[ii].height >> 1);

		depth[ii] = src_gray_cp.at<uchar>(xc[ii],yc[ii]);

		cout << xc[ii];
		cout << "\t";
		cout << yc[ii];
		cout << "\t";
		cout << depth[ii];
		cout << "\n";
	}

	//Converting to BitValues
	for(int ii = 0; ii <contours.size(); ii++)
	{

		intToBin(_x1,(xtl[ii]));
		intToBin(_y1,(ytl[ii]));
		intToBin(_x2,(xbr[ii]));
		intToBin(_y2,(ybr[ii]));
		intToBin(_dp,(depth[ii]));
		cout << xtl[ii] << '\n';
		for (int jj = 10; jj >= 0; jj--)
		{
			cout << _x1[jj];
		}
		cout << '\n';
		cout << ytl[ii] << '\n';
		for (int jj = 10; jj >= 0; jj--)
		{
			cout << _y1[jj];
		}
		cout << '\n';
		cout << xbr[ii] << '\n';
		for (int jj = 10; jj >= 0; jj--)
		{
			cout << _x2[jj];
		}
		cout << '\n';
		cout << ybr[ii] << '\n';
		for (int jj = 10; jj >= 0; jj--)
		{
			cout << _y2[jj];
		}
		cout << '\n';
		cout << depth[ii] << '\n';
		for (int jj = 10; jj >= 0; jj--)
		{
			cout << _dp[jj];
		}
		cout << '\n';
	}


	//cout << yOne;
	cout << "\n";
	cout << "End of Binary";
	//cout << x2 << '\n' << y2 << '\n';
	//cout << dp << '\n' << '\n';

	//Show in window
	/*
	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	imshow("Contours",drawing);
	namedWindow("Overlay", CV_WINDOW_AUTOSIZE);
	imshow("Overlay", src_gray);
	*/
}

void opening (int, void*)
{

	element = getStructuringElement(morph_elem, Size(2*morph_size+1,2*morph_size+1), Point(morph_size,morph_size));

	morphologyEx( src_gray, opIm, operation, element);
	//imshow( window_name, opIm);
}

void intToBin(bool* bin, int num)
{
	cout << "In Function\n\n";
	for(int ii = 9; ii >= 0; ii--)
	{
		if(num >= ((pow(2,ii))))
		{
			bin[ii] = true;
			num = num - (pow(2,ii));
		}
		else
		bin[ii] = false;
	}
}
