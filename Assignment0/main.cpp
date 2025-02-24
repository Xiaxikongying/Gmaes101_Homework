#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;
// 给定一个点 P=(2,1), 将该点绕原点先逆时针旋转 45?，再平移 (1,2), 计算出变换后点的坐标（要求用齐次坐标进行计算）

int main()
{
    // 创建一个点
    Vector3f P(2, 1, 1);

    // 创建一个旋转矩阵
    Matrix3f R;
    R << cos(M_PI / 4), -sin(M_PI / 4), 0,
        sin(M_PI / 4), cos(M_PI / 4), 0,
        0, 0, 1;

    // 创建一个平移矩阵
    Matrix3f T;
    T << 1, 0, 1,
        0, 1, 2,
        0, 0, 1;

    P = T * R * P;
    cout << P << endl;
    return 0;
}

void test()
{
    // 创建一个点
    Vector3f P(2, 1, 1);
    // 创建一个旋转 平移矩阵
    Matrix3f M;
    M << cos(M_PI / 4), -sin(M_PI / 4), 1,
        sin(M_PI / 4), cos(M_PI / 4), 2,
        0, 0, 1;

    P = M * P;
    cout << P << endl;
}

void test_Games101()
{
    // Basic Example of cpp
    cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    cout << a << endl;
    cout << a / b << endl;
    cout << sqrt(b) << endl;
    cout << acos(-1) << endl;
    cout << sin(30.0 / 180.0 * acos(-1)) << endl;

    // Example of vector
    cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f w(1.0f, 0.0f, 0.0f);
    // vector output
    cout << "Example of output \n";
    cout << v << endl;
    // vector add
    cout << "Example of add \n";
    cout << v + w << endl;
    // vector scalar multiply
    cout << "Example of scalar multiply \n";
    cout << v * 3.0f << endl;
    cout << 2.0f * v << endl;

    // Example of matrix
    cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i, j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    cout << "Example of output \n";
    cout << i << endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v
    cout << "test1" << endl;
}