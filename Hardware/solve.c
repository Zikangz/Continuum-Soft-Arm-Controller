#include "stm32f10x.h"                  // Device header
 
// 计算系数矩阵的逆
void matrix_inverse(double matrix[2][2], double inverse[2][2]) {
    // 计算式子
    double A = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    inverse[0][0] = matrix[1][1] / A;
    inverse[0][1] = -matrix[0][1] / A;
    inverse[1][0] = -matrix[1][0] / A;
    inverse[1][1] = matrix[0][0] / A;
}
/*
//定义一个包含多个成员的结构体，然后让函数返回这个结构体。
struct Result {
    double x;
    double y;
};
*/ 
// 解二元一次方程组
void solve_equation(double A, double B, double C, double D, double E, double F, double results[]) {
    // 系数矩阵
    double matrix[2][2] = {{A, B}, {D, E}};
    // 结果矩阵
    double result[2] = {C, F};
    double inverse[2][2];
 
    matrix_inverse(matrix, inverse);
 
    // 计算结果

	results[0] = inverse[0][0] * result[0] + inverse[0][1] * result[1];
    results[1] = inverse[1][0] * result[0] + inverse[1][1] * result[1];
 
}
/* 
int main() {
    // 实例：2x + 3y = 7; 4x - 2y = 5
    double A = 2, B = 3, C = 7;
    double D = 4, E = -2, F = 5;
 
    struct Result result = solve_equation(A, B, C, D, E, F);
 
    return 0;
}
*/
