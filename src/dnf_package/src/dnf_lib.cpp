#pragma once
#include <torch/torch.h>
#include "../include/dnf_1d.hpp"

// Include ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/logging.hpp> //For "get_logger"

void DNFinit(){ //Torch debugging
    torch::Tensor myTensor = torch::rand({2,3});
    //std::cout << "Here is the example tensor: " << myTensor << std::endl; //debug, cout only works when running the node only

    // Create DNF
    DNF_1D my_dnf(10, true); //Create DNF with 10 dimensions
    torch::Tensor input = torch::rand({10});

    // Step DNF
    my_dnf.step(input, 0.1);

    // Get output
    torch::Tensor output = my_dnf.get_output();
}

std::vector<int> HSVFromRGB(int R, int G, int B){

    // Sanity check
    if (R > 255 || G > 255 || B > 255 || R < 0 || G < 0 || B < 0){
      RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "RGB values must be between 0 and 255, are: %d %d %d" , R, G, B);
        return {-1,-1,-1};
    }

    int h = -1 , s = -1, v = -1;
    double min, max, delta;

    min = R < G ? R : G;
    min = min  < B ? min  : B;

    max = R > G ? R : G;
    max = max  > B ? max  : B;

    v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        s = 0;
        h = 0; // undefined, maybe nan?
        std::vector<int> HSV = {h,s,v};
        return HSV;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        s = 0.0;
        h = 0.0;                            // its now undefined
        std::vector<int> HSV = {h,s,v};
        return HSV;
    }
    if( R >= max )                           // > is bogus, just keeps compilor happy
        h = ( G - B ) / delta;        // between yellow & magenta
    else
    if( G>= max )
       h = 2.0 + ( B - R ) / delta;  // between cyan & yellow
    else
        h = 4.0 + ( R - G ) / delta;  // between magenta & cyan

    h *= 60.0;                              // degrees

    if( h < 0.0 )
        h += 360.0;

    std::vector<int> HSV = {h,s,v};

    return HSV;

  }

  std::vector<int> RGBFromHSV(int H, int S, int V){

    // Sanity check
    if( H > 360 || H < 0 || S > 100 || S < 0 || V > 100 || V < 0){
      RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "HSV values must be between 0 and 360, 0 and 100, 0 and 100, are: %d %d %d", H, S, V);
      return {-1,-1,-1};
    }

    double      hh, p, q, t, ff; 

    long        i;
    int R = -1, G = -1, B = -1;

    if(S <= 0.0) {       // < is bogus, just shuts up warnings
        R = V;
        G = V;
        B = V;
        return {R,G,B};
    }
    hh = H;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = V * (1.0 - S);
    q = V * (1.0 - (S * ff));
    t = V * (1.0 - (S * (1.0 - ff)));

    switch(i) {
      case 0:
          R = V;
          G = t;
          B = p;
          break;
      case 1:
          R = q;
          G = V;
          B = p;
          break;
      case 2:
          R = p;
          G = V;
          B = t;
          break;

      case 3:
          R = p;
          G = q;
          B = V;
          break;
      case 4:
          R = t;
          G = p;
          B = V;
          break;
      case 5:
      default:
          R = V;
          G = p;
          B = q;
          break;
    }

    std::vector<int> RGB = {R,G,B};
    return RGB;
  }