#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    /* 
    * PA 0
    */
    // TO DO: Define point P
    Eigen::Vector3f P(2.0f,1.0f,1.0f);
    // TO DO: Define rotation matrix M
    Eigen::Matrix3f M;
    M << cos(M_PI/4), -sin(M_PI/4), 1,
         sin(M_PI/4), cos(M_PI/4), 2,
         0, 0, 1;
    // TO DO: M * P
    Eigen::Vector3f P2 = M * P;
    std::cout << "P2 = \n" << P2 << std::endl;
    return 0;
}