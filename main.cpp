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


using std::experimental::optional;
using std::pair;
using std::numeric_limits;

using cv::Mat;
using cv::Point;

using namespace boost::numeric::ublas;

using Real = float;


#define ASSUMED_DEGREE 30

#define MINIMAL_TRIANGLE_AREA 1000

#define DEBUG_MODE

//#define TEST_MODE

enum Side {
    Right,
    Left
};


template<typename T1, typename T2>
optional<T2> operator>>(optional<T1> a, std::function< optional<T2>(T1)> f) {
    if(a) return f(*a);
    return optional<T2>{};
}


std::pair<Real, Real> findEquiationCoeffs(cv::Vec4i points){

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

    return std::make_pair(x, y);
}


void filterBadLines(std::vector<cv::Vec4i>& lines){
    auto isBadLine = [](const cv::Vec4i& line) -> bool{
        // We think it is bad when it is more then ASSUMED_DEGREE - 1
        auto [k, b] = findEquiationCoeffs(line);
//        cout << "k: " << k << " b: " << b << endl;
        double tangent = tan(M_PI * (Real)(ASSUMED_DEGREE + 1)/180);
        if (k < 0){
            if (k > -tangent){
                return true;
            }
        }
        if (k > 0){
            if (k > tangent){
                return true;
            }
        }
        return false;
//        int x1 = line[0];
//        int y1 = line[1];
//        int x2 = line[2];
//        int y2 = line[3];
//
//        int catheter1 = abs(x1 - x2);
//        int catheter2 = abs(y1 - y2);
//
//        if (catheter2 == 0){
//            return false;
//        }
////        cout << (1/sqrt(3)) << endl;
////        cout << catheter1/catheter2 << endl;
//        if ((Real)catheter2/(Real)catheter1 < (1/sqrt(3))){
//            cout << (Real)catheter1/(Real)catheter2 << endl;
//            return true;
//        }
//
//        return false;

    };

    std::remove_if(lines.begin(), lines.end(), isBadLine);

    auto isOnOneLine = [](const cv::Vec4i& line1, const cv::Vec4i& line2){
        int Ax = line1[0];
        int Ay = line1[1];
        int Bx = line1[2];
        int By = line1[3];

        int Cx1 = line2[0];
        int Cy1 = line2[1];

        int Cx2 = line2[2];
        int Cy2 = line2[3];
        // Are of triangle is: [ Ax * (By - Cy) + Bx * (Cy - Ay) + Cx * (Ay - By) ] / 2
        float area1 = (Ax * (By - Cy1) + Bx * (Cy1 - Ay) + Cx1 * (Ay - By)) / 2;
        float area2 = (Ax * (By - Cy2) + Bx * (Cy2 - Ay) + Cx2 * (Ay - By)) / 2;

        if (area1 < MINIMAL_TRIANGLE_AREA && area2 < MINIMAL_TRIANGLE_AREA){
            return true;
        }
        return false;
    };


    if (lines.size() < 2){
        throw std::exception();
    }

    auto iter = std::begin(lines) + 1;
    for(; iter != std::end(lines);) {
        if(isOnOneLine(lines[0], *iter)){
            lines.erase(iter);
        } else {
            ++iter;
            break;
        }
    }
}


Mat prepareBeforeLinesFinding(const Mat& image){
    Mat tempImg, destImg;
    cv::cvtColor(image, tempImg, cv::COLOR_BGR2BGRA);
    cv::Canny(tempImg, destImg, 1000, 10, 3);
    return destImg;
}


std::vector<cv::Vec4i> findLines(Mat& image){

    int minLineLength = 200;
    int maxLineGap = 25;

    std::vector<cv::Vec4i> lines;

    cv::HoughLinesP(image, lines, 1, M_PI/180, 100, minLineLength, maxLineGap);

    return lines;
}


Mat generateMatrix(const Mat& source, int beta_){
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

    return transfo;
}


Mat transformation(const Mat& source, int beta_){

    auto transfo = generateMatrix(source, beta_);

    Mat destination;

    cv::warpPerspective(source, destination, transfo, source.size(), cv::INTER_CUBIC);

    return destination;
}


void fixImage(const Mat& image, std::function<void (Mat&, int)> callback){

    auto specialImg = prepareBeforeLinesFinding(image);
#ifdef DEBUG_MODE
    draw(specialImg);
#endif

    int defaultBeta = 90;
    int beta;

    auto lines = findLines(specialImg);
#ifdef DEBUG_MODE
    auto tempImg = image;
    for(auto&l : lines){
        line( tempImg, Point(l[0], l[1]),
              Point(l[2], l[3]), cv::Scalar(0, 255, 0), 2, 8 );
    }
    draw(tempImg);
#endif
    assert(lines.size() > 1);
    filterBadLines(lines);
#ifdef DEBUG_MODE
    for(auto&l : lines){
        line( tempImg, Point(l[0], l[1]),
              Point(l[2], l[3]), cv::Scalar(0, 255, 255), 2, 8 );
    }
    draw(tempImg);
#endif

    auto findSide = [](cv::Vec4i line, int x) -> Side {
        if(line[0] > x && line[2] > x){
            return Side::Left;
        }
        return Side::Right;
    };

    auto [x, y] = findIntersectionPoint(lines)
            .value_or(std::make_pair(0, 0));

    Side prevSide = findSide(lines[0], x);

    for (int stepUpDown = -1; stepUpDown < 2; stepUpDown += 2) {
        beta = defaultBeta;

        for (unsigned i = 0; i < ASSUMED_DEGREE; ++i) {

            beta += stepUpDown;

            std::vector<cv::Vec4i> transformedLines(lines.size());

            std::transform(std::begin(lines), std::end(lines),
                           std::begin(transformedLines),
                           [=](cv::Vec4i& line) -> cv::Vec4i {
                               std::vector<cv::Point2f> points;
                               points.emplace_back(cv::Point2f(line[0], line[1]));
                               points.emplace_back(cv::Point2f(line[2], line[3]));

                               auto m = generateMatrix(specialImg, beta); // In each iteration ??

                               std::vector<cv::Point2f> newPoints;

                               cv::perspectiveTransform(points, newPoints, m);
                               return cv::Vec4i(
                                       {
                                               int(newPoints[0].x),
                                               int(newPoints[0].y),
                                               int(newPoints[1].x),
                                               int(newPoints[1].y)
                                       }
                               );
                           }
            );
            filterBadLines(transformedLines); // TODO: ????
            auto [x, y] = findIntersectionPoint(transformedLines)
                    .value_or(std::make_pair(0, 0));

#ifdef DEBUG_MODE
            auto destination = transformation(image, beta);
            line( destination, Point(transformedLines[0][0], transformedLines[0][1]),
                  Point(transformedLines[0][2], transformedLines[0][3]), cv::Scalar(0, 255, 0), 2, 8 );
            line( destination, Point(transformedLines[1][0], transformedLines[1][1]),
                  Point(transformedLines[1][2], transformedLines[1][3]), cv::Scalar(0, 255, 0), 2, 8 );
            line( destination, Point(transformedLines[1][0], transformedLines[1][1]),
                  Point(x, y), cv::Scalar(255, 255, 0), 2, 8 );
            line( destination, Point(transformedLines[0][0], transformedLines[0][1]),
                  Point(x, y), cv::Scalar(255, 255, 0), 2, 8 );
            draw(destination);
            cout << "Current: " << x << ", beta: " << beta << endl;
#endif // DEBUG_MODE

            Side currSide = findSide(transformedLines[0], x);

            if (currSide != prevSide){
                goto RESULT;
            }

            prevSide = currSide;
        }
    }

    RESULT:

    auto destination = transformation(image, beta);
    callback(destination, beta);

}


int main(int argc, char** argv) {

#ifdef DEBUG_MODE
//    std::string imgName("../tested_data/image0.JPG");
//    std::string imgName("../data/IMG_0048.JPG");
    std::string imgName("../tested_data/IMG_0047.JPG");
    cout << "DEBUG MODE ON!" << endl;
#else
#ifdef TEST_MODE
//    std::string imgName("../tested_data/imageedit_6_2688227020.jpg");
    std::string imgName("../data/image0.JPG");
#else
    assert(argc == 3);

    if (strcmp(argv[1], "--data") != 0){
        cout << strcmp(argv[0], "--data") << endl;
        std::abort();
    }

    std::string imgName(argv[2]);
#endif // TEST_MODE
#endif // DEBUG_MODE
#ifdef TEST_MODE
    Mat image = getImage(imgName);

    auto printWrapped = [](Mat& , int beta) -> void {
        std::cout << beta << std::endl;
    };
    fixImage(image, printWrapped);

#else
    cout << imgName << endl;

    Mat image = getImage(imgName);

    auto drawWrapped = [](Mat& image, int i = 0) -> void {
        draw(image);
    };

    draw(image);

    fixImage(image, drawWrapped);


    return EXIT_SUCCESS;
#endif
}