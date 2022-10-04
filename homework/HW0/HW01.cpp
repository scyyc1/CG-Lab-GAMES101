#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>

using namespace std;

#define DEG2RAD(n) n*(M_PI/180);

int main(){
    Eigen::Vector3d P(2,1,1);
    Eigen::Vector3d O(0,0,1);
    Eigen::MatrixXd Rotation(3,3);
    Roataion << cos(DEG2RAD(45)), -sin(DEG2RAD(45)), 0,
                sin(DEG2RAD(45)), cos(DEG2RAD(45)), 0,
                0,0,1;
    Eigen::MatrixXd Translation(3,3);
    Translation << 1,0,1,
                   0,1,2,
                   0,0,1;
    cout << Translation * (Rotation * P);

    return 0;
}