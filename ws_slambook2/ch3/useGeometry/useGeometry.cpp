#include <iostream>
#include <cmath>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

int main(int argc,char **argv){
    /*
    -    旋转矩阵（3×3）：Eigen::Matrix3d
    -    旋转向量（3×1）：Eigen::AngleAxisd
    -      欧拉角（3×1）：Eigen::Vector3d
    -      四元数（4×1）：Eigen::Quaterniond
    - 欧式变换矩阵（4×4）：Eigen::Isometry3d
    -    仿射变换（4×4）：Eigen::Affine3d
    -    射影变换（4×4）：Eigen::Projective3d
    */

    Matrix3d rotation_matrix = Matrix3d::Identity();
    // 创建轴角实例
    AngleAxisd rotation_vector(M_PI / 4,Vector3d(0,0,1));
    cout.precision(3);
    cout << "rotation_vector.matrix() = \n" << rotation_vector.matrix() << endl;
    rotation_matrix = rotation_vector.toRotationMatrix();
    cout << "\nrotation_matrix = \n" << rotation_matrix << endl;

    Vector3d v(1,0,0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "\n(1,0,0) after rotation (by angle axis) : " << v_rotated.transpose() <<endl;

    v_rotated = rotation_matrix * v;
    cout << "\n(1,0,0) after rotation (by rotation_matrix) : " << v_rotated.transpose() <<endl;

    // 欧拉角
    Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0); // 2 1 0 表示ZYX顺序
    cout << "\nyaw pitch roll : " << euler_angles.transpose() << endl; // 偏航、俯仰、滚转

    // 欧式变换矩阵可以使用Eigen::Isometry
    Isometry3d T = Isometry3d::Identity();  // 虽然称为3d，但是是4×4的矩阵
    T.rotate(rotation_vector);
    T.pretranslate(Vector3d(1,3,4));
    cout << "\nTransform matrix = \n" << T.matrix() << endl;

    // 用变换矩阵进行坐标变换
    Vector3d v_transformed = T * v;
    cout << "\nv transformed = " << v_transformed.transpose() << endl;

    //仿射变换、射影变换可以使用 Eigen::Affine3d 和 Eigen::Projective3d
    // 四元数
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "\nquaternion[x,y,z,w] from rotation vector = " << q.coeffs().transpose() << endl;
    q = Quaterniond(rotation_matrix);
    cout << "\nquaternion[x,y,z,w] from rotation matrix = " << q.coeffs().transpose() << endl;
    // 用四元数旋转一个向量
    v_rotated = q * v;
    cout << "\n(1,0,0) after quaternion : " << v_rotated.transpose() << endl;
    // 使用常规乘法
    cout << "\nshould be equal to : " << (q * Quaterniond(0,1,0,0) * q.inverse()).coeffs().transpose() << endl;

    // 注意，四元数在使用前需要进行归一化
    q.normalize();
}