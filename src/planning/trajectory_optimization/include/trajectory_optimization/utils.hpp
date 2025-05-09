#include <Eigen/Dense>
using namespace Eigen;

/**
 * @brief Calculates the difference between consecutive elements of the same column
 */
MatrixXd diff_col(MatrixXd E) { 
    MatrixXd E1 = E.block(0, 0, E.rows()-1, E.cols());
    MatrixXd E2 = E.block(1, 0, E.rows()-1, E.cols());
    return E2 - E1;
}

/**
 * @brief Calculates the cumulative sum of the vector elements at each index
 */
VectorXd cumsum(VectorXd v){
    int n = v.size(); VectorXd res(n);

    res(0) = v(0);
    for(int i = 1; i < n; i++){
        res(i) = v.head(i+1).sum();
    }

    return res;
}

/**
 * @brief Calculates numerical gradient of the vector f
 */
VectorXd gradient(VectorXd f){
    int n = f.size();
    VectorXd grad(n);

    grad(0) = f(1)-f(0);
    for(int i = 1; i < n-1; i++){
        grad(i) = (f(i+1)-f(i-1))/2;
    }
    grad(n-1) = f(n-1)-f(n-2);

    return grad;
}