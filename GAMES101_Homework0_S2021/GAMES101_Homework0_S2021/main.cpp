#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

int main()
{

    Eigen::Vector3f P(2.0f, 1.0f, 1.0f);
    // angle = pi/4
    double angle = 0.25 * acos(-1);
    Eigen::Matrix3f T;
    T << cos(angle), -sin(angle), 1.0,
        sin(angle), cos(angle), 2.0,
        0.0, 0.0, 1.0;
    std::cout << "The answer is \n";
    std::cout << T * P << std::endl; // do rotation R first, then do translation T

    return 0;
}