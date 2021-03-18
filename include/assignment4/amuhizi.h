/* 
  04-801 Cognitive Robotics Assignment 4
  --------------------------------------
 
  David Vernon
  1 March 2017

  Ported to ROS and OpenCV 3.3
  DV 14 March 2021

*/

#define ROS
 
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <ctype.h>
#include <iostream>
#include <string>


#ifndef ROS
   #include <conio.h>
#else
   #include <ros/ros.h>
   #include <ros/package.h>
   #include <sys/select.h>
   #include <termios.h>
   #include <stropts.h>
   #include <sys/ioctl.h>
#endif

//opencv
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp> 

#define TRUE  1
#define FALSE 0
#define MAX_FILENAME_LENGTH  200
#define ARROW_HEAD_SIZE      (10.0)
#define ARROW_HEAD_ANGLE     (3.14159 / 4.0)
#define MAX_NUMBER_OF_BLOCKS 100
#define MIN_CONTOUR_LENGTH   150
#define MIN_CONTOUR_AREA     1000
#define RED_TOLERANCE        20 // hues in the range 180-RED_TOLERANCE are deemed to be red, and are assigned a value of 0

using namespace std;
using namespace cv;

/* function prototypes go here */

void rgb2hsi(unsigned char red, unsigned char green, unsigned char blue, float *hue, float *saturation, float *intensity);
void drawCrossHairs(Mat hough, int x, int y, int size, int r, int g, int b, int weight);
void drawArrowedLine(Mat hough, int i, int j, float magnitude_value, float phase_value, int red, int green, int blue, int weight);
float lineLength(Point2f p1, Point2f p2);

void prompt_and_exit(int status);
void prompt_and_continue();

void getAngle(cv::Point point, cv::Point center, float *thetha);
void radToDeg(float rad, float *deg);
void getCenter(cv::Mat *src, std::vector<std::vector<cv::Point>> contours,std::vector<cv::Point> *centers, std::vector<cv::Point> *points_on_arcLine);
void distance(int x1, int y1, int x2, int y2, float *distance);
void ContourExtraction(cv::Mat src, std::vector<std::vector<cv::Point>> *contours, int thresholdValue);

#ifdef ROS
   int _kbhit();
#endif
   
