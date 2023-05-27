#ifndef DNF_UTILS_CPP
#define DNF_UTILS_CPP

/*
 * Description:     A collection of utility functions for the DNF package
 *
 * Author:			Jaccob Fløe Jeppsen & Erik Lindby
 * Institution: 	University of Southern Denmark
 *
 * Creation date:	13-02-2021
 * 
 * Notes:           This file is included in the dnf_2d.cpp and dnf_pubsub_function.cpp files.
 *                  Therefore to prevent multiple definitions of the functions, all functions must be defined as inline.
 *                  This is done by adding the inline keyword before the function definition.
 *                  The header guard is not sufficient to prevent multiple definitions of the functions.
 */

//#pragma once
#include <torch/torch.h>
#include "../include/dnf_1d.hpp"

// Include ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/logging.hpp> //For "get_logger"

#include <iostream>
#include <fstream>
#include <string>
#include <glob.h> //For globbing purposes
#include <filesystem> //For filesystem get_path current purposes
// #include <ament_index_cpp/get_package_share_directory.hpp> //For getting the package path
// may throw ament_index_cpp::PackageNotFoundError exception

enum enCol          { RED = 0,      GREEN = 1,  BLUE = 2 };
enum enNorm         { NONE = 0,     NORM = 1,   COLNORM = 2};
enum enLearn        { ML = 0,       HEBBIAN = 1};
enum enSuppression  { NO_SUPP = 0,  SUPP = 1};

inline void printTensor(torch::Tensor tensor){
    // Check if 1D or 2D tensor given
    if (tensor.sizes()[1] == 0){
        // 1D tensor
        RCLCPP_INFO(rclcpp::get_logger("PrintTensor"), "Tensor is 1D");
        for (int i = 0; i < tensor.sizes()[0]; i++){
            RCLCPP_INFO(rclcpp::get_logger("PrintTensor"), "tensor[%d] = %f", i, tensor[i].item<float>());
        }
        return;
    } 
    // 2D tensor
    RCLCPP_INFO(rclcpp::get_logger("PrintTensor"), "Tensor is 2D");
    for (int i = 0; i < tensor.sizes()[0]; i++){
        for (int j = 0; j < tensor.sizes()[1]; j++){
            RCLCPP_INFO(rclcpp::get_logger("PrintTensor"), "tensor[%d][%d] = %f", i, j, tensor[i][j].item<float>());
        }
    }
}

inline void DNFinit(){ //Torch debugging
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

inline std::vector<float> HSVFromRGB(float R, float G, float B){

    // Sanity check
    if (R > 255 || G > 255 || B > 255 || R < 0 || G < 0 || B < 0){
      RCLCPP_INFO(rclcpp::get_logger("HSVFromRGB"), "RGB values must be between 0 and 255, are: %d %d %d" , R, G, B);
        return {-1,-1,-1};
    }

    float h = -1 , s = -1, v = -1;
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
        std::vector<float> HSV = {h,s,v};
        return HSV;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        s = 0.0;
        h = 0.0;                            // its now undefined
        std::vector<float> HSV = {h,s,v};
        return HSV;
    }
    if( R >= max )                           // > is bogus, just keeps compilor happy
        h = ( G - B ) / delta;        // between yellow & magenta
    else if( G>= max )
       h = 2.0 + ( B - R ) / delta;  // between cyan & yellow
    else
        h = 4.0 + ( R - G ) / delta;  // between magenta & cyan

    h *= 60.0;                              // degrees

    if( h < 0.0 )
        h += 360.0;

    std::vector<float> HSV = {h,s,v};

    return HSV;

  }

  inline std::vector<int> RGBFromHSV(int H, int S, int V){
    // Sanity check
    if( H > 360 || H < 0 || S > 100 || S < 0 || V > 100 || V < 0){
      RCLCPP_INFO(rclcpp::get_logger("RGBFromHSV"), "HSV values must be between 0 and 360, 0 and 100, 0 and 100, are: %d %d %d", H, S, V);
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

inline void write2DTensorCSV(torch::Tensor tensor, std::string log_path, std::string filename){
   // if( tensor.size(0) == 0 && tensor.size(1) == 0){
    //    RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "Tensor is empty, not writing to file");
   //    return;
    //}
    
    //RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "Beginning to write tensor to file");
    std::string file_path = log_path + filename;
    RCLCPP_INFO(rclcpp::get_logger("write2DTensorCSV"), "Writing to: %s", file_path.c_str());

    // Write tensor to file
    std::ofstream file;
    file.open(file_path);
    if(file.fail()){
        RCLCPP_INFO(rclcpp::get_logger("write2DTensorCSV"), "Failed to open file");
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("write2DTensorCSV"), "Tensor has sizes: %d %d", tensor.sizes()[0], tensor.sizes()[1]);
    // Check dimensions of tensor
    if(tensor.sizes()[1] == 0){ // 1D tensor
        for (int i = 0; i < tensor.size(0); i++){
            file << tensor[i].item<float>() << ",";
        }
    } else{ //2D tensor
        for (int i = 0; i < tensor.size(0); i++){
            for (int j = 0; j < tensor.size(1); j++){
                file << tensor[i][j].item<float>() << ",";
            }
            file << std::endl;
        }
    }
    
    file.close();
    RCLCPP_INFO(rclcpp::get_logger("write2DTensorCSV"), "Finished writing tensor to file");
}

inline torch::Tensor read2DTensorCSV(std::string log_path, std::string filename){
    if( filename == ""){
        RCLCPP_INFO(rclcpp::get_logger("read2DTensorCSV"), "No filename given");
        return torch::zeros({1,1});
    }
    std::string file_path = log_path + filename;

    std::ifstream file;
    file.open(file_path);
    if(file.fail()){
        RCLCPP_INFO(rclcpp::get_logger("read2DTensorCSV"), "Failed to open file %s", file_path.c_str());
        return torch::zeros({1,1});
    }
    std::vector<std::vector<double>> values;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream in(line);
        std::vector<double> row;
        std::string field;
        while (std::getline(in, field, ',')) {
            row.push_back(std::stof(field));
        }
        values.push_back(row);
    }
    file.close();

    // Loop through vector<vector<float>> values and print each element
    // for (size_t i = 0; i < values.size(); i++){
    //     for (size_t j = 0; j < values[i].size(); j++){
    //         RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "values[%d][%d] = %f", i, j, values[i][j]);
    //     }
    // }

    // Copying into a tensor
    long int n = values.size();
    long int m = values[0].size();
    auto options = torch::TensorOptions().dtype(at::kDouble);
    auto tensor = torch::zeros({n,m}, options);
    for (int i = 0; i < n; i++)
        tensor.slice(0, i,i+1) = torch::from_blob(values[i].data(), {m}, options);
    
    RCLCPP_INFO(rclcpp::get_logger("read2DTensorCSV"), "Finished reading tensor from file");
    return tensor;
}

inline torch::Tensor normalizeColumns(torch::Tensor& matrix) {
  int numColumns = matrix.size(1);  // Get the number of columns in the matrix
  torch::Tensor normalizedMatrix = torch::zeros_like(matrix);  // Create a new tensor with zeros of the same shape
  
  for (int col = 0; col < numColumns; ++col) {
    // Select the column by slicing the matrix
    auto column = matrix.slice(/*dim=*/1, col, col + 1);
    
    // Find the maximum value in the column
    auto [maxValue, maxIdx] = column.max(0);

    // If maxValue is less than or equal to one, then the column is already normalized. Also avoids dividing by zero
    if (maxValue.item<float>() <= 1.0) {
      normalizedMatrix.slice(/*dim=*/1, col, col + 1).copy_(column);
      continue;
    }
    
    // Normalize each element in the column by dividing it by the maximum value
    auto normalizedColumn = column / maxValue;
    
    // Copy the normalized column to the corresponding column in the new tensor
    normalizedMatrix.slice(/*dim=*/1, col, col + 1).copy_(normalizedColumn);
  }
  return normalizedMatrix;
}

#endif // DNF_UTILS_CPP
