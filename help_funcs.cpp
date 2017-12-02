#include <iostream>

#include <cv.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>


using std::string, std::cout, std::endl;
using std::vector;

using cv::Mat;

Mat getImage(string& imgFileName){

    Mat image = cv::imread(imgFileName, cv::IMREAD_COLOR);

    if (image.empty()) {
        cout <<  "Could not open or find the image" << endl ;
        std::abort();
    }

    cv::resize(image, image, cv::Size(1500, 750));

    return image;
}


void draw(Mat& image){
    cv::namedWindow("Img");
    cv::resizeWindow("Img", 200, 200);
    cv::imshow("Display window", image);

    cv::waitKey(0);
}