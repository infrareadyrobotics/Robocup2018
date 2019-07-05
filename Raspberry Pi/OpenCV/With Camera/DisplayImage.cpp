#include <stdio.h>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <fstream>

using namespace cv;
using namespace std;

Mat image;
int i = 0;

fstream uart("/dev/ttyUSB0");

void track(Mat img, int h1, int s1, int v1, int h2, int s2, int v2, char type)
{
  vector<vector<Point>> contours;

  inRange(img, cv::Scalar(h1, s1, v1), cv::Scalar(h2, s2, v2), img); //Detect objects that are only within the specified range
  findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<vector<Point>> contours_poly(contours.size());
  vector<Rect> boundRect(contours.size());
  vector<Point2f> center(contours.size());
  vector<float> radius(contours.size());

  for (size_t i = 0; i < contours.size(); i++)
  {
    approxPolyDP(contours[i], contours_poly[i], 3, true);
    boundRect[i] = boundingRect(Mat(contours_poly[i]));         //Fit a rectangle to each contour
    minEnclosingCircle(contours_poly[i], center[i], radius[i]); //Fit a circle to each contour
    if (radius[i] > 16)
    {
      uart << /*'\x02' <<*/ type << center[i] << radius[i] << "      " << boundRect[i] << '\xA' << '\xD';
      //cout <<'\x02' << type << center[i] << radius[i];
    }
  }
}

int main(int argc, char **argv)
{ // write (fd,test,sizeof(test));
  setNumThreads(4);
  VideoCapture camera(0);

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Thread count : " << getNumThreads() << endl;

  if (!camera.isOpened())
  {
    cerr << "ERROR: Unable to open the camera" << endl;
    return 0;
  }

  for (;;)
  {
    camera >> image;
    i++;
    //cout << i << '\n';
    cout.flush();

    cvtColor(image, image, COLOR_BGR2HSV);

    track(image, 0, 40, 40, 20, 255, 200, 'C'); //cyan
    track(image, 0, 40, 40, 20, 255, 200, 'Y'); //yellow
    track(image, 0, 40, 40, 20, 255, 200, 'O'); //orange
  }
}