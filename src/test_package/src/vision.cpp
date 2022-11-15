#pragma once
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <vector>


using std::placeholders::_1; // MIght not be necessary


float averageHueFromCircle(cv::Mat image, cv::Vec3f circle){
  cv::Mat cvtImage;
  image.copyTo(cvtImage);
  cv::cvtColor(image,cvtImage, cv::COLOR_BGR2HSV); //converting BGR to HSV 

  cv::Mat roi = cvtImage(cv::Range(circle[1]-circle[2], circle[1]+circle[2]), cv::Range(circle[0]-circle[2], circle[0]+circle[2]));
   // ROI from minX & maxX, minY & maxY. x coordinate - radius = minX   x coordinate + radius = maxX
  
  //Calculate mean color of ROI
  float temp = 0;
  float insideCounter = 0;
  for (size_t row = 0; row < image.rows; row++){
    for (size_t col = 0; col < image.cols; col++){
      if( ( sqrt( pow(row-circle[0],2) + pow(col-circle[1],2) ) <= circle[2] ) ) {
        temp += image.at<cv::Vec3b>(row, col)[0]; //* roi.at<cv::Vec3b>(row, col)[1];
        insideCounter++;
      }
    } 
  }
  return temp/insideCounter;
}

std::vector<cv::Vec6f> detect_circles(cv::Mat &image){

    std::vector<cv::Vec3f> circles;
    std::vector<cv::Vec6f> circleInfo;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 25,
                  1, 30 // (min_radius & max_radius) to detect larger circles
    );

    //Draw center and outline of all detected circles 
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec6f circleInfoTemp;
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);

        
        //Circle color
        cv::Vec3b color = image.at<cv::Vec3b>(center);
        float avgHue = averageHueFromCircle(image, c);

        // circle center
        cv::circle( image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        
        // circle outline
        int radius = c[2];
        cv::circle( image, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
        
        // Create output vector 
        circleInfoTemp[0] = c[0];
        circleInfoTemp[1] = c[1];
        // circleInfoTemp[2] = c[2];
        circleInfoTemp[2] = avgHue;
        circleInfoTemp[3] = color[0];
        circleInfoTemp[4] = color[1];
        circleInfoTemp[5] = color[2];
        circleInfo.push_back(circleInfoTemp);

        cv::putText(image, //target image
              std::to_string(i),    // Text
              center,
              cv::FONT_HERSHEY_DUPLEX,
              0.6,
              CV_RGB(225, 255, 255),  // Font color
              2);
    }
  cv::putText(image, //target image
                "Detected " + std::to_string(circles.size() ) + " circles" ,    // Text
                cv::Point(10, image.rows / 10),   // Top-left position
                cv::FONT_HERSHEY_DUPLEX,
                0.6,
                CV_RGB(225, 255, 255),  // Font color
                2);

  //Return detected circles
  // cv::cvtColor(image,image, cv::COLOR_RGB2HSV);
  return circleInfo;

}


