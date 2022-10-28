#pragma once
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <vector>


using std::placeholders::_1; // MIght not be necessary

std::vector<cv::Vec3f> detect_circles(cv::Mat &image){

    std::vector<cv::Vec3f> circles;
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
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);

        // circle center
        cv::circle( image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        
        // circle outline
        int radius = c[2];
        cv::circle( image, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }
  //Return detected circles

  return circles;

}

