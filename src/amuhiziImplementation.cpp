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

