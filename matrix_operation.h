#ifndef MATRIX_OPERATION
#define MATRIX_OPERATION
#include<vector>
using std::vector;
//the matrix operation of multiplication
vector <vector<double> > matrix_multiply(vector<vector<double> > arrA, vector<vector<double> > arrB);
//the matrix operation of addition
vector <vector<double> > matrix_add(vector<vector<double> > arrA, vector<vector<double> > arrB);
//the matrix operation of transposition
vector <vector<double> > matrix_Trans(vector<vector<double> > arrA);
//the matrix operation of inversion
double matrix_Det(vector<vector<double> >arrA);
vector <vector<double> > matrix_inv(vector<vector<double> >arrA);
//the matrix operation of subtraction
vector <vector<double> > matrix_sub(vector<vector<double> > arrA, vector<vector<double> > arrB);

#endif // MATRIX_OPERATION
