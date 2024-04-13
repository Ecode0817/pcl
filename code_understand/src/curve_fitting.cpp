#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// 多项式拟合函数
VectorXd polyfit(const VectorXd& x, const VectorXd& y, int degree) {
    MatrixXd A(x.size(), degree + 1);
    VectorXd y_tmp = y;

    for (int i = 0; i < x.size(); i++) {
        double xi = 1.0;
        for (int j = 0; j <= degree; j++) {
            A(i, j) = xi;
            xi *= x[i];
        }
    }

    // 求解线性方程组
    VectorXd coeffs = A.fullPivLu().solve(y_tmp);

    return coeffs;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " input.pcd output.pcd" << endl;
        return -1;
    }

    // 读取输入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file \n");
        return -1;
    }

    // 提取点云数据
    VectorXd x(cloud->size()), y(cloud->size()), z(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        x[i] = cloud->points[i].x;
        y[i] = cloud->points[i].y;
        z[i] = cloud->points[i].z;
    }
// 找到点云数据中的最小和最大坐标值对应的索引
int min_index = 0, max_index = 0;
double min_distance = std::numeric_limits<double>::max();
double max_distance = std::numeric_limits<double>::min();
for (size_t i = 0; i < cloud->size(); ++i) {
    double distance = x[i]*x[i] + y[i]*y[i] + z[i]*z[i];
    if (distance < min_distance) {
        min_distance = distance;
        min_index = i;
    }
    if (distance > max_distance) {
        max_distance = distance;
        max_index = i;
    }
}

// 起点和终点即为最小和最大坐标值对应的点
double start_x = x[min_index];
double start_y = y[min_index];
double start_z = z[min_index];
double end_x = x[max_index];
double end_y = y[max_index];
double end_z = z[max_index];


    // 拟合曲线
    VectorXd coeffs_x = polyfit(z, x, 5); // x = f(z), 5次多项式拟合
    VectorXd coeffs_y = polyfit(z, y, 5); // y = g(z), 5次多项式拟合

    cout << "Coefficients for x = f(z): ";
    for (int i = 0; i <= 5; ++i) {
        cout << coeffs_x[i] << " ";
    }
    cout << endl;

    cout << "Coefficients for y = g(z): ";
    for (int i = 0; i <= 5; ++i) {
        cout << coeffs_y[i] << " ";
    }
    cout << endl;

    // 生成100个均匀分布的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr curve(new pcl::PointCloud<pcl::PointXYZ>);
    curve->width = 100;
    curve->height = 1;
    curve->points.resize(curve->width * curve->height);
    for (int i = 0; i < 100; ++i) {
        double alpha = i / 99.0;
        double z_val = start_z + alpha * (end_z - start_z);
        double x_val = 0.0, y_val = 0.0;
        for (int j = 0; j <= 5; ++j) {
            x_val += coeffs_x[j] * pow(z_val, j);
            y_val += coeffs_y[j] * pow(z_val, j);
        }
        curve->points[i].x = x_val;
        curve->points[i].y = y_val;
        curve->points[i].z = z_val;
    }

    // 保存拟合的曲线点云
    pcl::io::savePCDFileASCII(argv[2], *curve);
    cout << "Curve saved to " << argv[2] << endl;

    return 0;
}
