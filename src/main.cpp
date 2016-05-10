/*
* Intel Cornell Cup
* Authors: Brandon Contino & Neel Kowdley <nkowdley@gmail.com>
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
//#include "SimpleGPIO.h"

//define the max iterations
#define MAX_TIME 100
#define THRESH 144
#define BITS 10 //Total bits - 1
#define DISPLAY_TIMING 1 //1 - Enable Timing Output | 0 - Disable Timing Output

using namespace std;
using namespace cv;

//Image Declarations
Mat src; Mat src_gray; Mat src_gray_cp; Mat opIm; Mat element; Mat threshold_output; Mat overlay;
std::string filename;

//Threshold Characteristics
int thresh = THRESH;
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
int count_real;

//Function Declarations
void thresh_contour(int, void*, sl::zed::Camera*);
void opening(int, void*);
void intToBin(bool*, int);
void save_img(sl::zed::Camera* zed, cv::Mat disp, int );

//Timing Declarations
double startUpTime;
double imgLoadTime;
double cvTime;
//GPIO Configuration
//GPIO test
//const int GPIO_LED = 57;

int main(int argc, char **argv) {
	//GPIO test
	//gpio_export(GPIO_LED);
	//gpio_set_dir(GPIO_LED, OUTPUT_PIN);

	//Initialize a counter and a timer
	count_real = 0;
	
	double startUpT1 = (double)getTickCount(); 

	//ZED Initialization
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
	int ConfidenceIdx = 1000;
	int width = zed->getImageSize().width;
	int height = zed->getImageSize().height;
	cv::Mat disp(height, width, CV_8UC3);
	//cv::Size DisplaySize(720, 404);
	//cv::Mat dispDisplay(DisplaySize, CV_8UC3);

	//Jetson only. Execute the calling thread on core 2
	sl::zed::Camera::sticktoCPUCore(2);

	// DisparityMap filtering
	//zed->setDispReliability(reliabilityIdx);
	zed->setConfidenceThreshold(ConfidenceIdx);
	bool res = zed->grab(dm_type);
	sleep(2);
	res = zed->grab(dm_type);
	double startUpT2 = (double)getTickCount();
	while(count_real < MAX_TIME) {
		printf("count: %d\n",count_real);
		if (res) {
			cout << "Something went wrong\n";
			return 1;
		}
		
		double imgLoadT1 = (double)getTickCount();		

		slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DISPARITY)).copyTo(disp);
		//cv::resize(disp, dispDisplay, DisplaySize);
		//begin saving the disparity image
		//Don't write the image anymore, uncomment this to write out the file
		save_img(zed,disp,count_real);

		double imgLoadT2 = (double)getTickCount();
		double cvT1 = (double)getTickCount();

		//Coordinate Initialization
		_x1 = (bool*) calloc (BITS,sizeof(bool));
		_y1 = (bool*) calloc (BITS,sizeof(bool));
		_x2 = (bool*) calloc (BITS,sizeof(bool));
		_y2 = (bool*) calloc (BITS,sizeof(bool));
		_dp = (bool*) calloc (BITS,sizeof(bool));

		//////////////////////////////////////////////////NEW LINES BETWEEN////////////////////////////////////////////////////////
		//src = imread("/home/ubuntu/Desktop/1271_27.png", 1);
		//cvtColor( src, src_gray, CV_BGR2GRAY);
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
		cvtColor( disp, src_gray, CV_BGR2GRAY );

		//disp.copyTo(src_gray);
		//save_img(zed,src_gray,999);
		src_gray_cp= src_gray.clone();

		opening(0,0);
		thresh_contour(0,0,zed);
		double cvT2 = (double)getTickCount();
		
		if(DISPLAY_TIMING) {
			if(count_real == 0) {
				startUpTime = (startUpT2 - startUpT1)/getTickFrequency();
				imgLoadTime = (imgLoadT2 - imgLoadT1)/getTickFrequency();
				cvTime = (cvT2 - cvT1)/getTickFrequency();		
				}			
			else {
				startUpTime = (startUpTime + (startUpT2 - startUpT1)/getTickFrequency())/2;
				imgLoadTime = (imgLoadTime + (imgLoadT2 - imgLoadT1)/getTickFrequency())/2;
				cvTime = (cvTime + (cvT2 - cvT1)/getTickFrequency())/2;
			}		
		}

		count_real++;
		res = zed->grab(dm_type);	
	}

	if(DISPLAY_TIMING) {
		cout << "\nOver "<< MAX_TIME << ": ";
		cout << "\nStart Up Time is: " << startUpTime;			
		cout << "\nAverage Image Load Time is: " << imgLoadTime;
		cout << "\nAverage CV Runtime is is: " << cvTime << "\n";
	}

	/*//wait to exit
	int x;
	scanf("%d",&x);*/
	return 0;
}

void save_img(sl::zed::Camera* zed, cv::Mat disp, int count) {
	filename.clear();
	filename = std::string(("ZEDDisparity") + std::to_string(count) + std::string(".png")); //create the filename
	cv::Mat dispSnapshot;
	disp.copyTo(dispSnapshot);
	//std::time_t result = std::time(nullptr);	
	//char filename= count;
	//strTmp = std::localtime(&result);
	//Don't write the image anymore, uncomment this to write out the file
	cv::imwrite(filename, dispSnapshot);
	//delete *strTmp;
	//cout << "\n" + filename + "\n";
}
/*Begin Helper Functions*/
void thresh_contour(int, void*,sl::zed::Camera* zed)
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
	int depth [(contours.size()-1)]; //CHANGED FROM UNSIGNED INT TO INT
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
		//printf("xc[ii]=%d,yc[ii]=%d\n",xc[ii],yc[ii]);
		//printf("rows=%d,cols=%d\n",src_gray_cp.size().height,src_gray_cp.size().width);
		depth[ii] = src_gray_cp.at<uchar>(xc[ii],yc[ii]);
		//printf("uchar_thingy=%d\n", src_gray_cp.at<uchar>(xc[ii],yc[ii]));
		cout << xc[ii];
		cout << "\t";
		cout << yc[ii];
		cout << "\t";
		cout << depth[ii];
		cout << "\n";
	}

	//Converting to BitValues
	for(int ii = 0; ii < contours.size(); ii++)
	{
		intToBin(_x1,(xtl[ii]));
		intToBin(_y1,(ytl[ii]));
		intToBin(_x2,(xbr[ii]));
		intToBin(_y2,(ybr[ii]));
		intToBin(_dp,(depth[ii]));
			//cout << xtl[ii] << '\n';
		for (int jj = BITS; jj >= 0; jj--) 
		{
			cout << _x1[jj];
		}
		cout << '\t';
		//cout << ytl[ii] << '\n';
		for (int jj = BITS; jj >= 0; jj--) 
		{
			cout << _y1[jj];
		}
		cout << '\t';
		//cout << xbr[ii] << '\n';
		for (int jj = BITS; jj >= 0; jj--) 
		{
			cout << _x2[jj];
		}
		cout << '\t';
		//cout << ybr[ii] << '\n';
		for (int jj = BITS; jj >= 0; jj--) 
		{
			cout << _y2[jj];
		}
		cout << '\t';
		//cout << depth[ii] << '\n';
		 for (int jj = BITS; jj >= 0; jj--) 
		{
			cout << _dp[jj];

		}
		cout << '\n';
	}

	//Save Image
	save_img(zed,drawing,900+count_real);
	save_img(zed,src_gray,1000+count_real);


}

void opening (int, void*)
{

	element = getStructuringElement(morph_elem, Size(2*morph_size+1,2*morph_size+1), Point(morph_size,morph_size));

	morphologyEx( src_gray, opIm, operation, element);

}

void intToBin(bool* bin, int num)
{
	for(int ii = BITS; ii >= 0; ii--)

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
