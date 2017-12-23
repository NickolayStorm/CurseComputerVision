/* Copyright (C) 2017 Nikolai Pakhtusov - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: chesterlanduk@gmail.com
 */

#include <vector>
// TODO: actually it is not an experimental/optional;
// TODO: In c++17 it is just <optional> but it is not working
#include <experimental/optional>

#include <cv.h>
#include <opencv2/imgcodecs.hpp>

#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "help_funcs.cpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>


using std::experimental::optional;
using std::pair;
using std::numeric_limits;

using cv::Mat;
using cv::Point;

using namespace boost::numeric::ublas;

using Real = double;


template<typename T1, typename T2>
optional<T2> operator>>(optional<T1> a, std::function< optional<T2>(T1)> f) {
    if(a) return f(*a);
    return optional<T2>{};
}


std::pair<Real, Real> findEquiationCoeffs(cv::Vec4i points){

    cout << points << endl;

    int x1, y1, x2, y2;

    x1 = points[0];
    y1 = points[1];
    x2 = points[2];
    y2 = points[3];

    Real k = ((Real) (y2 - y1)) /
             ((Real) (x2 - x1));
    Real b = - ((Real) (x1 * (y2 - y1)) /
                (Real) (x2 - x1))
             + y1;
    cout << "k: " << k << " b: " << b << endl;
//    assert(fabs(y1 - (k * x1 + b)) < numeric_limits<Real>::epsilon());
//    assert(fabs(y2 - (k * x2 + b)) < numeric_limits<Real>::epsilon());


    return std::make_pair(k, b);
};


// точки схода
// birds eye view

optional<pair<Real, Real>>
findIntersectionPoint(const std::vector<cv::Vec4i>& lines){

    if (lines.size() < 2){
        return std::experimental::nullopt;
    }

    // TODO: get special lines (instead of two first)
    auto [k1, b1] = findEquiationCoeffs(lines[0]);
    auto [k2, b2] = findEquiationCoeffs(lines[1]);

    // TODO: use special library for it (for example LAPACK)
    //    matrix<Real> A = identity_matrix<Real>(2);
    //    A(0, 0) = k1; A(0, 1) = b1;
    //    A(0, 0) = k1; A(0, 1) = b1;
    Real x = (b2 - b1) / (k1 - k2);
    Real y = k1 * x + b1;

//    assert(fabs((y - ((k2 * x) + b2))) < numeric_limits<Real>::epsilon());

    return std::make_pair(x, y);
}


std::vector<cv::Vec4i> findLines(Mat& image){
    Mat tempImg, destImg;

    cv::cvtColor(image, tempImg, cv::COLOR_BGR2Lab);

    cv::Canny(tempImg, destImg, 1000, 10, 3);

    int minLineLength = 200;
    int maxLineGap = 10;

    std::vector<cv::Vec4i> lines;

    cv::HoughLinesP(destImg, lines, 1, M_PI/180, 100, minLineLength, maxLineGap);

    return lines;
}


Mat transformation(Mat source, int beta_){
    int alpha_=90;
    int f_ = 500, dist_ = 500;

    Real f, dist;
    Real alpha, beta, gamma;
    beta = ((Real)beta_ - 90)*M_PI/180;
    f = (Real) f_;
    dist = (Real) dist_;

    cv::Size taille = source.size();
    Real w = (Real)taille.width, h = (Real)taille.height;

    // Projection 2D -> 3D matrix
    Mat A1 = (cv::Mat_<Real>(4,3) <<
            1, 0, -w/2,
            0, 1, -h/2,
            0, 0,    0,
            0, 0,    1);

    // Rotation matrices around the Y axe
    Mat R = (cv::Mat_<Real>(4, 4) <<
            cos(beta), 0, -sin(beta), 0,
            0,         1,          0, 0,
            sin(beta), 0,  cos(beta), 0,
            0,         0,          0, 1);

    // Translation matrix on the Z axis change dist will change the height
    Mat T = (cv::Mat_<Real>(4, 4) <<
            1, 0, 0,    0,
            0, 1, 0,    0,
            0, 0, 1, dist,
            0, 0, 0,    1);       // Camera Intrisecs matrix 3D -> 2D
    // 500 was f; f was from 1 to 2000 (magic-magic amm)
    Mat A2 = (cv::Mat_<Real>(3,4) <<
                                  500, 0, w/2, 0,
            0, 500, h/2, 0,
            0, 0,   1, 0);

    Mat transfo = A2 * (T * (R * A1));

    Mat destination;

    warpPerspective(source, destination, transfo, taille, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);

    return destination;
}


int main() {

    std::string imgName("../image0.jpg");

    Mat image = getImage(imgName);

    auto lines = findLines(image);

    optional<pair<Real, Real>> maybePoint = findIntersectionPoint(lines);

    auto [x, y] = maybePoint.value_or(std::make_pair(0, 0) );
    line( image, Point(lines[1][2], lines[1][3]),
          Point(x, y), cv::Scalar(0, 255, 0), 3, 8 );

    line( image, Point(lines[0][2], lines[0][3]),
          Point(x, y), cv::Scalar(0, 255, 0), 3, 8 );

    Mat destination;
    Mat source = image;

    int beta_ = 90;

    cv::namedWindow("Result", 1);

    cv::createTrackbar("Beta", "Result", &beta_, 180);

    while(true) {

        auto destination = transformation(source, beta_);

        // Output
        imshow("Result", destination);

        lines = findLines(destination);

        optional<pair<Real, Real>> maybePoint = findIntersectionPoint(lines);

        auto [x, y] = maybePoint.value_or(std::make_pair(0, 0));

        cout <<  x << " when f was: " << beta_;

        int iKey = cv::waitKey(5000);

        //if user press 'ESC' key
        if (iKey == 27)
        {
            break;
        }
        findIntersectionPoint(lines);

    }


    return EXIT_SUCCESS;
}


// Code fo right and full bird eye transformation:
//// Projection 2D -> 3D matrix
//Mat A1 = (cv::Mat_<Real>(4,3) <<
//                                1, 0, -w/2,
//        0, 1, -h/2,
//        0, 0,    0,
//        0, 0,    1);
//
//// Rotation matrices around the X,Y,Z axis
//Mat RX = (cv::Mat_<Real>(4, 4) <<
//                                 1,          0,           0, 0,
//        0, cos(alpha), -sin(alpha), 0,
//        0, sin(alpha),  cos(alpha), 0,
//        0,          0,           0, 1);
//
//Mat RY = (cv::Mat_<Real>(4, 4) <<
//                                 cos(beta), 0, -sin(beta), 0,
//        0, 1,          0, 0,
//        sin(beta), 0,  cos(beta), 0,
//        0, 0,          0, 1);
//
//Mat RZ = (cv::Mat_<Real>(4, 4) <<
//                                 cos(gamma), -sin(gamma), 0, 0,
//        sin(gamma),  cos(gamma), 0, 0,
//        0,          0,           1, 0,
//        0,          0,           0, 1);
//
//// Composed rotation matrix with (RX,RY,RZ)
//Mat R = RX * RY * RZ;
//
//// Translation matrix on the Z axis change dist will change the height
//Mat T = (cv::Mat_<Real>(4, 4) <<           1, 0, 0, 0,           0, 1, 0, 0,           0, 0, 1, dist,           0, 0, 0, 1);       // Camera Intrisecs matrix 3D -> 2D
//Mat A2 = (cv::Mat_<Real>(3,4) <<
//                                f, 0, w/2, 0,
//        0, f, h/2, 0,
//        0, 0,   1, 0);
//
//// Final and overall transformation matrix
//Mat transfo = A2 * (T * (R * A1));