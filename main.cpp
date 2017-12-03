
#include <iostream>
#include <vector>
// TODO: actually it is not an experimental/optional;
// TODO: In c++17 it is just <optional> but it is not working
#include <experimental/optional>

#include <cv.h>
#include <opencv2/imgcodecs.hpp>

#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "help_funcs.cpp"



using std::string;
using std::cout, std::endl;

using std::experimental::optional;
using std::pair;
using std::numeric_limits;

using cv::Mat;
using cv::Point;

using namespace boost::numeric::ublas;

using Real = float;


optional<pair<Real, Real>>
findIntersectionPoint(const std::vector<cv::Vec4i>& lines){

    if (lines.size() < 2){
        return std::experimental::nullopt;
    }

    auto findCoeffs = [](cv::Vec4i points){

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

        assert(fabs(y1 - (k * x1 + b)) < numeric_limits<Real>::epsilon());
        assert(fabs(y2 - (k * x2 + b)) < numeric_limits<Real>::epsilon());


        return std::make_pair(k, b);
    };

    // TODO: get special lines (instead of two first)
    auto [k1, b1] = findCoeffs(lines[0]);
    auto [k2, b2] = findCoeffs(lines[1]);

    // TODO: use special library for it (for example LAPACK)
    //    matrix<Real> A = identity_matrix<Real>(2);
    //    A(0, 0) = k1; A(0, 1) = b1;
    //    A(0, 0) = k1; A(0, 1) = b1;
    //    A(0, 0) = k1; A(0, 1) = b1;
    Real x = (b2 - b1) / (k1 - k2);
    Real y = k1 * x + b1;

    assert(fabs((y - ((k2 * x) + b2))) < numeric_limits<Real>::epsilon());


    return std::make_pair(x, y);
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

    std::vector<cv::Vec4i> lines;

    cv::HoughLinesP(destImg, lines, 1, M_PI/180, 100, minLineLength, maxLineGap);

    cout << "Lines size: " << lines.size() << endl;

    for(auto &oneLine : lines){
        line( image, Point(oneLine[0], oneLine[1]),
              Point(oneLine[2], oneLine[3]), cv::Scalar(0, 0, 255), 3, 8 );
        cout << oneLine << endl;
    }

    // Now we use 2 lines only so they are blue
    line( image, Point(lines[0][0], lines[0][1]),
              Point(lines[0][2], lines[0][3]), cv::Scalar(255, 0, 0), 3, 8 );
    line( image, Point(lines[1][0], lines[1][1]),
          Point(lines[1][2], lines[1][3]), cv::Scalar(255, 0, 0), 3, 8 );

    draw(image);

    auto maybePoint = findIntersectionPoint(lines);


    return EXIT_SUCCESS;
}