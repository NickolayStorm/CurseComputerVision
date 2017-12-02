#include <iostream>
#include <cv.h>
#include <opencv2/imgcodecs.hpp>

#include <vector>

//#include <boost>

#include <boost>

#include "help_funcs.cpp"



using std::string, std::cout, std::endl;
using std::vector;

using cv::Mat;
using cv::Point;

// image0.jpg


void findIntersectionPoint(vector<cv::Vec4i> lines){


}


int main() {

    string imgName("../image0.jpg");

    Mat image = getImage(imgName);
    draw(image);

    Mat tempImg, destImg;

    cv::cvtColor(image, tempImg, cv::COLOR_BGR2Lab);
    draw(tempImg);

    cv::Canny(tempImg, destImg, 1000, 10, 3);
    draw(destImg);


    int minLineLength = 200;
    int maxLineGap = 10;

    vector<cv::Vec4i> lines;

    cv::HoughLinesP(destImg, lines, 1, M_PI/180, 100, minLineLength, maxLineGap);

    cout << "Lines size: " << lines.size() << endl;

    for(auto &oneLine : lines){
        line( image, Point(oneLine[0], oneLine[1]),
              Point(oneLine[2], oneLine[3]), cv::Scalar(0, 0, 255), 3, 8 );
        cout << oneLine << endl;
    }

    draw(image);



    return EXIT_SUCCESS;
}