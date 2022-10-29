#pragma once
#include <vector>

// std::string vec3fToString(std::vector<cv::Vec3f> vec){
    //     std::string vecStr;
    //     for (size_t i = 0; i < vec.size(); i++)
    //     {
    //         vecStr.append(std::to_string(vec[i][0])+','+std::to_string(vec[i][1])+','+std::to_string(vec[i][2])+";");
    //     }
    //     return vecStr;
    // }

    // int dividerCounter = 0; //Delete this later!
    // std::string testStr;

    // //std::vector<cv::Vec3f> 
    // void stringToVec3f(std::string str){
    //     dividerCounter = 0;
    //     std::vector<cv::Vec3f> vec;
    //     cv::Vec3f vec3f;
    //     size_t pos = 0;  
    //     std::string tempString1, tempString2; // define string temps  
        
    //     // use find() function to get the position of the delimiters  
    //     while ( ( pos = str.find(";") ) != std::string::npos)  
    //     {  
    //         tempString1 = str.substr(0, pos); // store the substring   
    //         str.erase(0, pos + 1);  /* erase() function store the current positon and move to next token. */  
    //         dividerCounter++; 
    //         // testStr.append(tempString1);
    //     }  

    //     while ( ( pos = tempString1.find(",") ) != std::string::npos)  
    //     {  
    //         tempString2 = tempString1.substr(0, pos); // store the substring   
    //         tempString1.erase(0, pos + 1);  /* erase() function store the current positon and move to next token. */  
    //         vec3f.append(std::stof(tempString2));
    //         // tempString2.append(tempString1);
    //     }  

    //     // return 3fvec;
    // }
