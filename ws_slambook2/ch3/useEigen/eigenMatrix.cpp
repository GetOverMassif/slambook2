#include <iostream>
using namespace std;

#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

#define MATRIX_SIZE 50

/*
* 本程序演示了Eigen基本类型的使用
*/

int main(int argc,char **argv){
    // Eigen中所有向量和矩阵都是Eigen::Matrix，它是一个模板类，前三个参数为：数据类型、行、列
    Matrix<float,2,3> matrix_23;
    Vector3d v_3d;  // 三维向量，实质上是一个Eigen::Matrix<double,3,1>
    Matrix<float,3,1> vd_3d;

    Matrix3d matrix_33 = Matrix3d::Zero();
    Matrix<double,Dynamic,Dynamic> matrix_dynamic;
    MatrixXd matrix_x;

    /******对Eigen库的操作******/
    matrix_23 << 1,2,3,4,5,6;
    cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;

    cout << "\nprint matrix 2x3: " << endl;
    for(int i = 0;i < 2;i++){
        for(int j = 0;j < 3;j++) cout << matrix_23(i,j) << "\t";
        cout << endl;
    }

    v_3d << 3,2,1;
    vd_3d << 4,5,6;
    cout << endl << "v_3d:\n" << v_3d << endl;
    cout << endl << "vd_3d:\n" << vd_3d << endl;

    // 矩阵的数据类型转换
    Matrix<double,2,1> result = matrix_23.cast<double>() * v_3d;
    cout << "[1,2,3;4,5,6]*[3,2,1]=[" << result.transpose() << "]^T" << endl; 
    Matrix<float,2,1> result2 = matrix_23 * vd_3d;
    cout << "[1,2,3;4,5,6]*[4,5,6]=[" << result2.transpose() << "]^T" << endl; 

    matrix_33 = Matrix3d::Random();
    cout << "\nrandom matrix: \n" << matrix_33 << endl;
    cout << "\ntranspose: \n" << matrix_33.transpose() << endl;
    cout << "\nsum: " << matrix_33.sum() << endl;
    cout << "\ntrace: " << matrix_33.trace() << endl;
    cout << "\ntimes 10: \n" << 10 * matrix_33 << endl;
    cout << "\ninverse: \n" << matrix_33.inverse() << endl;
    cout << "\ndet: " << matrix_33.determinant() << endl;

    // 特征值
    cout << "\nmatrix_33.transpose() * matrix_33=\n" << matrix_33.transpose() * matrix_33 << endl;
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "\nEigen values = \n" << eigen_solver.eigenvalues() << endl;
    cout << "\nEigen vectors(列为特征向量) = \n" << eigen_solver.eigenvectors() << endl;

    // 解方程
    // 方程形式： matrix_NN * x = v_Nd
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN
        = MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();  // 保证半正定
    Matrix<double,MATRIX_SIZE,1> v_Nd = MatrixXd::Random(MATRIX_SIZE,1);

    clock_t time_stt = clock();
    // 【Method1: 直接求逆】
    Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse() * v_Nd;
    cout << "\ntime of【normal inverse】is "
         << 100 * (clock()-time_stt)/(double)CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    // 【Method2: 矩阵分解来求解，如QR分解】
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "\ntime of【Qr decomposition】is "
         << 100 * (clock()-time_stt)/(double)CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    // 【Method3: 对于正定矩阵，还可以用cholesky分解解方程】
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "\ntime of【ldlt decomposition】is "
         << 100 * (clock()-time_stt)/(double)CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    return 0;
}