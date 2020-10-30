#include <string>
#include <fstream>
#include <iostream>
#include <math.h>
#include "opencv2/opencv.hpp"


int main(){
  
    std::string line;
    int i = 0;
    std::ifstream myfile("/home/peter/Documents/data/poses/00.txt");
    double x =0, y=0, z = 0;
    cv::Mat camera_pose = cv::Mat::eye(4, 4, CV_64F);

    if (myfile.is_open())
    {
        while (std::getline (myfile,line))
        {
            std::istringstream in(line);
            //cout << line << '\n';
            std::vector<double> t;
            for (int j=0; j<12; j++)  {
                in >> z ;
                t.push_back(z);
            }
            cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
            memcpy(T.data,t.data(),t.size()*sizeof(float));
            i++;
        }
        myfile.close();
    }

    else {
        std::cout << "Unable to open file";
        return 0;
    }

    return 0;
}