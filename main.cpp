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


#define ASSUMED_DEGREE 30

#define MINIMAL_TRIANGLE_AREA 1000


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
findIntersectionPoint(std::vector<cv::Vec4i>& lines){

    auto isBadLine = [](const cv::Vec4i& line) -> bool{
        // We think it is bad when it is more then ASSUMED_DEGREE
        auto [k, b] = findEquiationCoeffs(line);
//        cout << "k: " << k << " tan " << tan(M_PI * (Real)ASSUMED_DEGREE/(Real)180) << endl;
        if ((tan(-M_PI * (Real)ASSUMED_DEGREE/180)) > k > (tan(M_PI * (Real)ASSUMED_DEGREE/180))){
//            cout << "Removed" << endl;
            return false;
        }
        return true;
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

        cout << area1 << " " << area2 << endl;

        if (area1 < MINIMAL_TRIANGLE_AREA && area2 < MINIMAL_TRIANGLE_AREA){
            return true;
        }
        return false;
    };


    if (lines.size() < 2){
        return std::experimental::nullopt;
    }

    auto iter = std::begin(lines) + 1;
    for(; iter != std::end(lines); ++iter) {
        if(isOnOneLine(lines[0], *iter)){

            lines.erase(iter);
        } else {
            break;
        }
    }

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


Mat prepareBeforeLinesFinding(const Mat& image){
    Mat tempImg, destImg;
    cv::cvtColor(image, tempImg, cv::COLOR_BGR2Lab);
    cv::Canny(tempImg, destImg, 1000, 10, 3);
    return destImg;
}


std::vector<cv::Vec4i> findLines(Mat& image){

    int minLineLength = 200;
    int maxLineGap = 10;

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


void fixImage(const Mat& image, std::function<void (Mat&)> callback){

    auto specialImg = prepareBeforeLinesFinding(image);

    int defaultBeta = 90;
    int beta;

    auto lines = findLines(specialImg);
    auto [previous, y] = findIntersectionPoint(lines)
            .value_or(std::make_pair(0, 0));

    for (int stepUpDown = -1; stepUpDown < 2; stepUpDown += 2) {
        beta = defaultBeta;

        for (unsigned i = 0; i < ASSUMED_DEGREE; ++i) {

            beta += stepUpDown;

            std::vector<cv::Vec4i> newLines(lines.size());

            std::transform(std::begin(lines), std::end(lines),
                           std::begin(newLines),
                           [=](cv::Vec4i& line) -> cv::Vec4i {
                               std::vector<cv::Point2f> points;
                               points.emplace_back(cv::Point2f(line[0], line[1]));
                               points.emplace_back(cv::Point2f(line[2], line[3]));

                               auto m = generateMatrix(specialImg, beta);

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

            auto destination = transformation(specialImg, beta);

            auto [current, y] = findIntersectionPoint(newLines)
                    .value_or(std::make_pair(0, 0));

            cout << "Current: " << current << " previous: " << previous << ", beta: " << beta << endl;
            // If they have different signs
            if ((current < 0 && previous > 0) || (current > 0 && previous < 0)) {
                goto RESULT;
            }
            previous = current;
        }
    }

    RESULT:
        cout << "The best beta: " << beta << endl;
        auto destination = transformation(image, beta);
        draw(destination);
}


int main() {

    std::string imgName("../data/image1.jpg");
    Mat image = getImage(imgName);
    draw(image);

    auto callBack = [](Mat& image){
        draw(image);
    };

    fixImage(image, callBack);


    return EXIT_SUCCESS;
}