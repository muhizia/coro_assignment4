/* 
  04-801 Cognitive Robotics Assignment 4
  --------------------------------------
 
  David Vernon
  1 March 2018

  Ported to ROS and added Linux version of _kbhit
  DV 14 March 2021

*/

#include "assignment4/amuhizi.h"

void drawCrossHairs(Mat hough,  int x, int y, int size, int red, int green, int blue, int lineWeight) {

   Point pt1, pt2;

   pt1.x = cvRound(x + size/2);
   pt1.y = cvRound(y);
   pt2.x = cvRound(x - size/2);
   pt2.y = cvRound(y);
   line( hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);

   pt1.x = cvRound(x);
   pt1.y = cvRound(y + size/2);
   pt2.x = cvRound(x);
   pt2.y = cvRound(y - size/2);
   line( hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);
}



void drawArrowedLine(Mat hough, int i, int j, float magnitude_value, float phase_value, int red, int green, int blue, int lineWeight) {

   Point pt1, pt2;
   double theta;
   double i_offset;
   double j_offset;
   int i2, j2;
   int i3, j3;
  
   i_offset = magnitude_value * cos(phase_value);
   j_offset = magnitude_value * sin(phase_value);

   i2 = i + (int)(i_offset);
   j2 = j + (int)(j_offset);

   if ((i2 >= 0) && (i2 < hough.cols) && (j2 >= 0) && (j2 < hough.rows)) {

      pt1.x = i;
      pt1.y = j;
      pt2.x = i2;
      pt2.y = j2;
   
      line(hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);

      /* add arrow head */

      theta = phase_value + 3.14159 + ARROW_HEAD_ANGLE;
      i_offset = ARROW_HEAD_SIZE * cos(theta);
      j_offset = ARROW_HEAD_SIZE * sin(theta);

      i3 = i2 + (int)(i_offset);
      j3 = j2 + (int)(j_offset);

      pt1.x = i2;
      pt1.y = j2;
      pt2.x = i3;
      pt2.y = j3;

      line(hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);

      theta = phase_value + 3.14159 - ARROW_HEAD_ANGLE;
      i_offset = ARROW_HEAD_SIZE * cos(theta);
      j_offset = ARROW_HEAD_SIZE * sin(theta);;

      i3 = i2 + (int)(i_offset);
      j3 = j2 + (int)(j_offset);

      pt1.x = i2;
      pt1.y = j2;
      pt2.x = i3;
      pt2.y = j3;

      line(hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);
   }
}



float  lineLength(Point2f p1, Point2f p2) {

   return(sqrt( (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)));
}



// -----------------------------------------------------------------------------------------------
// rgb2hsi
//
// convert an RGB triple to a HSI triple
//
// The transform is based on "The Taming of the Hue, Saturation and Brightness Colour Space", Allan Hanbury, Proc. CVWW, [Hanbury02]
// 
// -----------------------------------------------------------------------------------------------

void rgb2hsi(unsigned char red, unsigned char green, unsigned char blue, float *hue, float *saturation, float *intensity){

	double y, h, h_star, c, c1, c2,  s, r, g, b; 
   int min =256;

   //  0 <= hue <= 2 pi
   //  0 <= saturation <= 1

   r = (float) red   / 256;
   g = (float) green / 256;
   b = (float) blue  / 256;

   y  = 0.2125 * r + 0.7154 * g + 0.0721 * b;
   c1 =          r - 0.5    * g - 0.5    * b;
   c2 =            - 0.8660 * g + 0.8660 * b;


   // chroma c: [0,1]

   c = sqrt(c1*c1 + c2*c2);


   // hue h: [0,360]

   if (c == 0) { // h and s are undefined
      *hue        = (float) 0;
      *saturation = (float) 0;
   }
   else {
      if(c2 <= 0) {
         h = acos (c1/c);
      }
      else {
         h = 2*3.14159  - acos (c1/c);
      }

      h = 360 * (h / (2 * 3.14159)); // convert to degrees


      // saturation: [0,1]

      h_star =  (int) h - (int) (60 * (  ((int) h) / 60));  // convert to interval 0,60


      s = (2 * c * sin( 2 * 3.14159 * ((120 - h_star) / 360.0))) / 1.73205;


      //*hue        = (float)  ((h / 360) * 2 * 3.14159); // convert to radians ... for the moment anyway
      *hue        = (float)  h;  
      *saturation = (float)  s;
   }

 	*intensity  = (float)  (r+g+b)/3;

  // printf("rgb2hsi: (%d, %d, %d) -> (%3.1f, %3.1f, %3.1f)\n", red, green, blue, *hue, *saturation, *intensity);

}

void prompt_and_exit(int status) {
   printf("Press enter to continue and close terminal ... \n");
   getchar();
   exit(status);
}


void prompt_and_continue() {
   printf("Press enter to continue ... \n");
   getchar();
}


void ContourExtraction(cv::Mat src, std::vector<std::vector<cv::Point>> *contours, int thresholdValue)
{
    cv::Mat src_gray;
    cv::Mat src_blur;
    cv::Mat detected_edges;
    
    bool debug = true;
    int ratio = 3;
    int kernel_size = 3;
    int filter_size;
    
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat thresholdedImage;
    filter_size = kernel_size * 4 + 1;
    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY) ;
    // multiplier must be even to ensure an odd filter size as required by OpenCV
    // this places an upper limit on gaussian_std dev of 7 to ensure the filter size < 31
    // which is the maximum size for the Laplacian operator
    GaussianBlur(src_gray, src_blur, cv::Size(filter_size,filter_size), kernel_size);
    Canny(src_gray, detected_edges,thresholdValue, thresholdValue*ratio, kernel_size ) ;

    
    
    cv::Mat canny_edge_image_copy = detected_edges.clone();
    // clone the edge image because findContours overwrites it
    /* see http://docs.opencv.org/2.4/modu1es/imgproc/doc/structura1 analysis and shape descriptors. html#findcontours */
    /* and http://docs.opencv.org/2.4/doc/tutoria1s/imgproc/shapedescriptors/find contours/ find contours. html */
    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    findContours (canny_edge_image_copy, *contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat contours_image = cv::Mat::zeros(src.size(), CV_8UC3);
    if(debug)
    {
        imshow("Gray", src_gray);
        imshow("Image brur", src_blur);
        imshow("Image detected_edges", detected_edges);
    }
}

void distance(int x1, int y1, int x2, int y2, int *distance)
{
   // Calculating distance between two points using two x,y coordinates.
   *distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}
void getCenter(cv::Mat *src, std::vector<std::vector<cv::Point>> contours,std::vector<cv::Point> *centers, std::vector<cv::Point> *points_on_arcLine){
    int centerX, centerY;
    bool debug = false;
    int distance_1, distance_2;
    std::vector<std::vector<cv::Point>> conPoints(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    cv::Point point_0, point_1, point_2, point_3;
    
    int area;
    float arc;
    
    for (int contour_number=0; (contour_number<(int)contours.size()); contour_number++) {
        cv::Scalar colour( rand()&0xFF, rand()&0xFF, rand()&0xFF ) ;    // use a random colour for each contour
        // calculating the area of the contours
        area = contourArea(contours[contour_number]);
        // calculating the perimeter of the contours function from openCV
        arc = arcLength(contours[contour_number], true);
        // Approximates a polygonal curve(s) with the specified precision.
        approxPolyDP(contours[contour_number], conPoints[contour_number], 0.03 * arc, true);
        boundRect[contour_number] = boundingRect(conPoints[contour_number]);
        if(debug) std::cout << "areas: " << area;
        if(area >= 2000) // Ignore detected contours that may be small which means they are not bricks. since the brick area is around 3000
        {
            // drawing bounding rectangle on each object in the picture.
            // rectangle(*src, boundRect[contour_number].tl(), boundRect[contour_number].br(), colour, 1);
            point_0 = cv::Point(conPoints[contour_number][0].x  , conPoints[contour_number][0].y);
            point_1 = cv::Point(conPoints[contour_number][1].x  , conPoints[contour_number][1].y);
            point_2 = cv::Point(conPoints[contour_number][2].x  , conPoints[contour_number][2].y);
            point_3 = cv::Point(conPoints[contour_number][3].x  , conPoints[contour_number][3].y);
            
            centerX = point_0.x/2 - point_2.x/2 + point_2.x;
            centerY = point_0.y/2 - point_2.y/2 + point_2.y;
            
            if(debug)
            {
                line(*src, cv::Point(centerX - 10.0, centerY), cv::Point(centerX + 10.0, centerY), cv::Scalar(0, 255, 255), 1);
                line(*src, cv::Point(centerX, centerY - 10.0), cv::Point(centerX, centerY + 10.0), cv::Scalar(0, 255, 255), 1);
                line(*src, point_0, point_1, cv::Scalar(0, 0, 0), 5);
                line(*src, point_1, cv::Point(point_1.x + centerX, point_1.y + centerY), cv::Scalar(255, 0, 0), 2);
            }
            // adding a center in vector of points
            centers->push_back(cv::Point(centerX, centerY));
            
            if(debug) std::cout << "Center x_0: " << centerX << " Center y_0: " << centerY<< std::endl;
            distance(point_0.x, point_0.y, point_1.x, point_1.y, &distance_1);
            distance(point_1.x, point_1.y, point_2.x, point_2.y, &distance_2);
            // check the smaller side on the found brick.
            if(distance_1 < distance_2)
            {
                points_on_arcLine->push_back(cv::Point((point_0.x + point_1.x) / 2, (point_0.y + point_1.y)/2));
                if(debug)
                    std::cout<<"Line " << contour_number + 1 << cv::Point(centerX, centerY)<< " ===> " << cv::Point((point_0.x + point_1.x) / 2, (point_0.y + point_1.y)/2) <<std::endl;
            }else{
                points_on_arcLine->push_back(cv::Point((point_0.x + point_3.x) / 2, (point_0.y + point_3.y)/2));
                if(debug)
                    std::cout<<"Line " << contour_number + 1 << cv::Point(centerX, centerY)<< " ===> " << cv::Point((point_0.x + point_3.x) / 2, (point_0.y + point_3.y)/2) <<std::endl;
            }
            if(debug)
            {
                line(*src, point_0, point_1, cv::Scalar(0, 255, 255), 1);
                line(*src, point_1, point_2, cv::Scalar(0, 0, 0), 1);
            }
        }
    }
}
void radToDeg(float rad, int *deg) // converting from radian to degree
{
    *deg = rad * (180.0/M_PI);
}
float degToRad(int deg) // converting from degree to radian
{
    return deg * (M_PI/180);
}
/**
 * getting the angle between two point and the horizontal.
 */
void getAngle(cv::Point point, cv::Point center, int *thetha)
{
    float deltaY, deltaX, rad;
    deltaY = point.y - center.y;
    deltaX = point.x - center.x;
    rad = abs(atan2(deltaY, deltaX));
    radToDeg(rad, thetha);
}

#ifdef ROS
/**
 Linux (POSIX) implementation of _kbhit().
 Morgan McGuire, morgan@cs.brown.edu
 */
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif

