#pragma once
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

using std::placeholders::_1; // Might not be necessary

/*
* Calculates color information inside detected circle (circle) in image 
*/
custom_msgs::msg::CircleInfo calcColorInfo(cv::Mat image, cv::Vec3f circle){ 
  std::vector<cv::Vec2f> colorInformation; //Each color information will contain mean and variance in floats for each colorspace
  // image.copyTo(cvtImage);
  //cv::cvtColor(image,cvtImage, cv::COLOR_BGR2HLS); //converting BGR to HSV 

  // cv::Mat roi = image(cv::Range(circle[1]-circle[2], circle[1]+circle[2]), cv::Range(circle[0]-circle[2], circle[0]+circle[2]));
   // ROI from minX & maxX, minY & maxY. x coordinate - radius = minX   x coordinate + radius = maxX

  //Calculate mean color of ROI
  std::vector<cv::Vec3b> pixels;
  float bgr_mean[3] = {0, 0, 0};
  for (int row = 0; row < image.rows; row++){
    for (int col = 0; col < image.cols; col++){
      // Check if pixel is inside circle
      if( ( sqrt( pow(col-circle[0],2) + pow(row-circle[1],2) ) <= circle[2]-2 ) ) { //Todo optimize sqrt, -2 because no outlier removal yet
        // Get pixel in image
        cv::Vec3b pixel = image.at<cv::Vec3b>(row,col);

        //Save pixel and add color value to sum for mean calc.
        bgr_mean[0] += pixel[0];
        bgr_mean[1] += pixel[1];
        bgr_mean[2] += pixel[2];
        pixels.push_back(pixel);

        // cv::Vec3b &color_point = image.at<cv::Vec3b>(row,col); // Get pixel (color)
        // color_point[0] = 80;                                   //Highlight pixels in ROI for debugging
        // color_point[1] = 80;                                   //Color value is set in image by reference
        // color_point[2] = 80;
      }
    }
  }

  // Calculate BGR mean
  bgr_mean[0] = bgr_mean[0] / pixels.size();
  bgr_mean[1] = bgr_mean[1] / pixels.size();
  bgr_mean[2] = bgr_mean[2] / pixels.size();
  float variance[3] = {0, 0, 0};
  
  for (size_t i = 0; i < pixels.size(); i++){
    variance[0] += (pixels[i][0] - bgr_mean[0]) * (pixels[i][0] - bgr_mean[0]); // Blue mean
    variance[1] += (pixels[i][1] - bgr_mean[1]) * (pixels[i][1] - bgr_mean[1]); // Green mean
    variance[2] += (pixels[i][2] - bgr_mean[2]) * (pixels[i][2] - bgr_mean[2]); // Red mean
  }
  variance[0] = variance[0] / ( pixels.size() - 1 );
  variance[1] = variance[1] / ( pixels.size() - 1 );
  variance[2] = variance[2] / ( pixels.size() - 1 );
  //float stdev[3] = { sqrt(variance[0]), sqrt(variance[1]), sqrt(variance[2]) };
  
  // Compile message
  auto circleInfo = custom_msgs::msg::CircleInfo();
  circleInfo.x            = circle[0];
  circleInfo.y            = circle[1];
  circleInfo.r            = circle[2];
  circleInfo.bgr_mean[0]  = bgr_mean[0];
  circleInfo.bgr_mean[1]  = bgr_mean[1];
  circleInfo.bgr_mean[2]  = bgr_mean[2];
  circleInfo.bgr_var[0]   = variance[0];
  circleInfo.bgr_var[1]   = variance[1];
  circleInfo.bgr_var[2]   = variance[2];
    
  return circleInfo;
}


// Function to transform image coordinates to real world coordinates
// cv::Point transform(cv::Point2d image_point, cv::Mat camera_matrix, cv::Mat dist_coeffs, cv::Mat rvec, cv::Mat tvec){
cv::Point transform(cv::Point image_point, cv::Mat camera_matrix, cv::Mat rvec, cv::Mat tvec){
  
  cv::Mat image_point_mat = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat real_point_mat = cv::Mat::zeros(3, 1, CV_64F);
  image_point_mat.at<double>(0, 0) = image_point.x;
  image_point_mat.at<double>(1, 0) = image_point.y;
  image_point_mat.at<double>(2, 0) = 1.0;
  cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_64F);
  cv::Rodrigues(rvec, rotation_matrix);
  // real_point_mat = rotation_matrix.inv() * (camera_matrix.inv() * image_point_mat - tvec);
  real_point_mat = rotation_matrix.inv() * (camera_matrix.inv() * image_point_mat - tvec);
  cv::Point  real_point;
  real_point.x = real_point_mat.at<double>(0, 0);
  real_point.y = real_point_mat.at<double>(1, 0);
  return real_point;
}

custom_msgs::msg::CircleInfoArr detect_circles(cv::Mat &image){
    std::vector<cv::Vec3f> circles;
    //std::vector<cv::Vec6f> circleInfo;
    auto circleInfoArr = custom_msgs::msg::CircleInfoArr();
    cv::Mat gray;
    cv::Mat og_image = image.clone();
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 25,
                  9, 30 // (min_radius & max_radius) to detect larger circles
    );


    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 
      277.19135641132203,
      0.0,
      160.5,
      0.0,
      277.19135641132203,
      120.5,
      0.0,
      0.0,
      1.0);

    cv::Mat dist_coeffs = (cv::Mat_<double>(5,1) << 0.0, 0.0, 0.0, 0.0, 0.0); // Assuming no lens distortion - Maybe add later

    cv::Mat rvec = (cv::Mat_<double>(3,1) <<  1.0, 1.0, 1.0);
    cv::Mat tvec = (cv::Mat_<double>(3,1) << 1, 1, 2.3);


    //Draw center and outline of all detected circles 
    for( size_t i = 0; i < circles.size(); i++ )
    {
      //auto circleInfo = custom_msgs::msg::CircleInfo();
      cv::Vec3i circle = circles[i];
      cv::Point center = cv::Point(circle[0], circle[1]);

      //Circle color
      //cv::Vec3b color = image.at<cv::Vec3b>(center);
      auto circleInfo = calcColorInfo(og_image, circle);
      circleInfoArr.circles.push_back(circleInfo);

      // circle center
      cv::circle( image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
      
      // circle outline
      int radius = circle[2];
      cv::circle( image, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);

 

    

      // cv::Point Q = transform(center, camera_matrix, rvec, tvec);

      

      cv::putText(image, //target image
            std::to_string(i), // std::to_string(Q.x) + ", " + std::to_string(Q.y),    // Text
            center, // Point
            cv::FONT_HERSHEY_DUPLEX,
            0.4,
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

  return circleInfoArr;

}





