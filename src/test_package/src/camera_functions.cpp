#include <memory>
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include<vector>


class camera_functions
{
private:
    std::vector<cv::Vec3f> circles;
    std::vector<cv::Point> coor_vec;
    std::vector<float> radius_vec;
    int n_circles;

public:
    camera_functions(/* args */);
    ~camera_functions();
    static cv::Mat hough_transform(cv::Mat image);
    static std::vector<cv::Point> get_circle_coordinates();
    static std::vector<float> get_circle_radius();
    static int get_n_circles();

};

camera_functions::camera_functions(/* args */)
{
}

camera_functions::~camera_functions()
{
}




cv::Mat camera_functions::hough_transform(cv::Mat image){

    //Reset information
    coor_vec.clear();
    radius_vec.clear();
    n_circles = 0;

    // Hough Transform

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, 1, 30 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    //Draw center and outline of all detected circles 
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        coor_vec.push_back(center);
        // circle center
        cv::circle( image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        radius_vec.push_back(radius);
        cv::circle( image, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);

        n_circles++;
    }
  //Return detected circles
  return image;

}

// ************* Get-functions ******************** //

// Coordinates contained in vector: [ cv::Point[x_1,y_1], cv::Point[x_2, y_2], ... cv::Point[x_n, y_n] ] 
std::vector<cv::Point> camera_functions::get_circle_coordinates(){ 
    return coor_vec;
}

// Radius contained in vector: [ r_1, r_2 ... r_n ]
std::vector<float> camera_functions::get_circle_radius(){
    return radius_vec;
}


