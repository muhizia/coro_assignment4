/* 
  04-801 Cognitive Robotics Assignment 4
  --------------------------------------
 
  The goal of this assignment is to develop an openCV vision application that can determine the pose 
  of coloured rectangular blocks placed on a flat surface. The pose of a block is given by the location 
  of the centroid and the angle between the major axis of the block and the horizontal.  

  It is assumend that the camera views the block from above (i.e. the principle ray of the camera is 
  orthogonal to the plane of the supporting surface).

  The application accepts image input from image files.  The image file names is provided in an input file. 
  The application writes the required data to an output file, i.e, for each block:
  
  - the coordinates of the centroid 
  - the angle subtended by the major axis and the horizontal in degrees
  
  The block poses are listed in increasing hue value, starting with red.


  Input
  The filenames of the images of the scenes containing coloured blocks.

  Output
  The output begin with the author's Andrew Id on a separate line
  This is followed, for each test case, by 
   
   - the filename 
   - the x and y coordinates in pixels and orientation in degrees of each block, enclosed in parentheses.  T
   
  There is just one filename and set of pose data per line. 

  Sample Input
  assignment4_0.png
  assignment4_1.png
  assignment4_2.png
  assignment4_3.png
  assignment4_14.png

  Sample Output
  amuhizi
  assignment4_0.jpg: (289, 160,  56) 
  assignment4_1.jpg: (227, 151,  21) 
  assignment4_2.jpg: (209, 146, 144) 
  assignment4_3.jpg: (162, 154,  79) (260, 142, 105) 
  assignment4_14.jpg: (259, 191,  80) (167, 179, 178) (352, 195, 180) 


  David Vernon
  1 March 2018

  Aristide Muhizi
  30 March 2020

  Audit Trail
  -----------
  Changed the range of orientation angle from -90 < theta <= 90 to 0 <= theta < 180
  DV 20 February 2020
  
  Exchanged the list of the picture name in the assignment4Input.txt
  Flipped right images to detect the position of the brick.
  Flipped left images to detect the position of the brick.
  Flipped down images to detect the position of the brick.
  
  Aristide Muhizi
  30 March 2021


  Ported to ROS
  DV 14 March 2021

*/

#include <assignment4/amuhizi.h> // replace studentid with your own student ID

int main()
{
   bool debug = true;

   const char input_filename[MAX_FILENAME_LENGTH] = "assignment4Input.txt";
   const char output_filename[MAX_FILENAME_LENGTH] = "assignment4Output.txt";
   char input_path_and_filename[MAX_FILENAME_LENGTH];
   char output_path_and_filename[MAX_FILENAME_LENGTH];
   char data_dir[MAX_FILENAME_LENGTH];
   char filename[MAX_FILENAME_LENGTH];
   char file_path_and_filename[MAX_FILENAME_LENGTH];

   FILE *fp_in, *fp_out;
   int end_of_file;

   Mat src;

   int row;
   int col;

   const char *input_window_name = "Input Image";

   int i;
   int x = 0;
   int y = 0;
   int theta = 0;
   int number_of_blocks = 0;

   cv::Mat imgGray, imgBlur, imgCanny;
   cv::Mat src_bgr;
   std::vector<std::vector<cv::Point>> contours;
   std::vector<cv::Point> centers;
   std::vector<cv::Point> arcLine_points;
   unsigned char red, green, blue;
   float hue, saturation, intensity;

   strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory

   strcat(data_dir, "/data/");

   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);

   strcpy(output_path_and_filename, data_dir);
   strcat(output_path_and_filename, output_filename);

   if ((fp_in = fopen(input_path_and_filename, "r")) == 0)
   {
      printf("Error can't open input %s\n", input_path_and_filename);
      prompt_and_exit(1);
   }

   if ((fp_out = fopen(output_path_and_filename, "w")) == 0)
   {
      printf("Error can't open output %s\n", output_path_and_filename);
      prompt_and_exit(1);
   }

   fprintf(fp_out, "amuhizi\n");

    
    std::vector<PIC_VALUES> pic_vals;


   namedWindow(input_window_name, CV_WINDOW_AUTOSIZE);
   
   do
   {

      end_of_file = fscanf(fp_in, "%s", filename);

      if (end_of_file != EOF)
      {
         if (debug)
            printf("%s\n", filename);

         number_of_blocks = 0;

         printf("Processing %s; press any key to continue ...\n", filename);

         strcpy(file_path_and_filename, data_dir);
         strcat(file_path_and_filename, filename);

         src = imread(file_path_and_filename, CV_LOAD_IMAGE_COLOR);

         if (src.empty())
         {
            cout << "can not open " << file_path_and_filename << endl;
            return -1;
         }

         imshow(input_window_name, src);
         waitKey(30);

         /* ----------------------------------------

         Solution to the assignment goes here ... 

         ----------------------------------------*/

         /* write results */

         /**
          * getting contours of the bricks in the pictures
         */
         ContourExtraction(src, &contours, 150);
         
         src_bgr = cv::imread(file_path_and_filename);
         /**
          * Getting centers of the bricks in the vector
         */
         getCenter(&src, contours, &centers, &arcLine_points);
         
         if (debug) printf("Number of contours %lu: \n", contours.size());
         fprintf(fp_out, "%s: ", filename);
         number_of_blocks = centers.size();
         for(int i = 0; i < number_of_blocks; i++)
         {
               getAngle(arcLine_points.at(i), centers.at(i), &theta);
               drawCrossHairs(src, centers.at(i).x, centers.at(i).y, 10, 255, 255, 0, 1);
               drawArrowedLine(src, centers.at(i).x, centers.at(i).y, 90, - degToRad(theta), 255, 255, 0, 1);
               getRGB(src_bgr, centers.at(i).x, centers.at(i).y, &red, &green, &blue);
               rgb2hsi(red, green, blue, &hue, &saturation, &intensity);
               hueMagnitue(&hue);
               pic_vals.push_back({centers.at(i).x, centers.at(i).y, theta, hue});
         }
         sort(pic_vals.begin(), pic_vals.end(), smallHue);
         for (auto pic : pic_vals){
               if(debug) printf("( %3d, %3d, %3d, %3f)", pic.x, pic.y, pic.theta, pic.hue);
               fprintf(fp_out, "(%3d, %3d, %3d) ", pic.x, pic.y, pic.theta);
         }
         if(debug) printf("\n");
         
         /**
          * clearing the picture value to use it on the next read picture.
         */

         pic_vals.clear();
         imshow( "Src", src);
         if(centers.size() != 0) centers.clear();
         if(arcLine_points.size() != 0) arcLine_points.clear();
         //cv::waitKey(0);
         /**
          * show image on the screen
         */
         imshow( "Src", src);
         fprintf(fp_out, "\n");
         /**
          *  After scanning one picture clear the vectors
          */
         if(centers.size() != 0) centers.clear();
         if(arcLine_points.size() != 0) arcLine_points.clear();
         do
         {
            waitKey(30);
         } while (!_kbhit());

         getchar(); // flush the buffer from the keyboard hit
      }

   } while (end_of_file != EOF);

   destroyWindow(input_window_name);

   fclose(fp_in);
   fclose(fp_out);

   return 0;
}
