#include "matrix_operation.h"
#include<iostream>
#include<vector>
#include<math.h>
using namespace std;

/*******the matrix operation of multiplication*******/
vector<vector<double> > matrix_multiply(vector<vector<double> > arrA, vector<vector<double> > arrB)
{
    //the rows of arrA
    int rowA = arrA.size();
    //the columns of arrA
    int colA = arrA[0].size();
    //the rows of arrB
    int rowB = arrB.size();
    //the columns of arrB
    int colB = arrB[0].size();
    //the result of matrix multiplication
    vector<vector<double> >  res;
    if (colA != rowB)
    {
        cout<<"Illegal multiplication!\n";
        return res;
    }
    else
    {
        //Initialization of res
        res.resize(rowA);
        for (int i = 0; i < rowA; ++i)
        {
            res[i].resize(colB);
        }

        //multiplication
        for (int i = 0; i < rowA; ++i)
        {
            for (int j = 0; j < colB; ++j)
            {
                for (int k = 0; k < colA; ++k)
                {
                    res[i][j] += arrA[i][k] * arrB[k][j];
                }
            }
        }
    }
    return res;
}


/******** the matrix operation of addition *******/
vector <vector<double> > matrix_add(vector<vector<double> > arrA, vector<vector<double> > arrB)
{
    //the rows of arrA
    int rowA = arrA.size();
    //the columns of arrA
    int colA = arrA[0].size();
    //the rows of arrB
    int rowB = arrB.size();
    //the columns of arrB
    int colB = arrB[0].size();
    //the result of matrix addition
    vector<vector<double> >  res;
    if (colA != colB || rowA !=rowB)
    {
        cout<<"Illegal Addition!\n";
        return res;
    }
    else
    {
        //Initialization of res
        res.resize(rowA);
        for (int i = 0; i < rowA; ++i)
        {
            res[i].resize(colB);
        }

        //addition
        for (int i = 0; i < rowA; ++i)
        {
            for (int j = 0; j < colA; ++j)
            {
                    res[i][j] += arrA[i][j] +arrB[i][j];

            }
        }
    }
    return res;
}

/******* the matrix operation of transposition *********/
vector <vector<double> > matrix_Trans(vector<vector<double> > arrA)
{
  int  rowA = arrA.size();
  int  colA = arrA[0].size();
    vector<vector<double> > res;
    res.resize(colA);
    for (int i = 0; i < colA; ++i)
    {
        res[i].resize(rowA);
    }

    for(int i=0;i<colA;i++)
        for(int j=0;j<rowA;j++)
            res[i][j]=arrA[j][i];
    return res;

}
 /**** the computation of determinant *****/
double matrix_Det(vector<vector<double> >arrA)
{

      int n=arrA.size();
      int r, c, m;
      int lop = 0;
      double result = 0;
      double mid = 1;
      if (n != 1)
      {
          lop = (n == 2) ? 1 : n;            //控制求和循环次数,若为2阶，则循环1次，否则为n次
          for (m = 0; m < lop; m++)
          {
              mid = 1;            //顺序求和, 主对角线元素相乘之和
              for (r = 0, c = m; r < n; r++, c++)
              {
                  mid = mid *arrA[r][c%n];
              }
              result += mid;
          }
          for (m = 0; m < lop; m++)
          {
              mid = 1;            //逆序相减, 减去次对角线元素乘积
              for (r = 0, c = n-1-m+n; r < n; r++, c--)
              {
                    mid = mid *arrA[r][c%n];
              }
              result -= mid;
          }
      }
      else
          result = arrA[0][0];
      return result;
      }

vector <vector<double> > matrix_inv(vector<vector<double> >arrA)
{
    vector <vector<double> > Adj;
    vector <vector<double> > arrA_inv;
   int f1=0;
   int f2=0;
    double Det=matrix_Det(arrA);
    int rows = arrA.size();
    int k=rows-1;

    double Adj_det;
    Adj.resize(k);
    for (int i = 0; i < k; ++i)
    {
        Adj[i].resize(k);
    }

    arrA_inv.resize(rows);
    for (int i = 0; i < rows; ++i)
    {
        arrA_inv[i].resize(rows);
    }
for(int x1=0;x1<rows;x1++)
  {
    for(int x2=0;x2<rows;x2++)
    {
    for (int m=0;m<rows;m++)
        for(int n=0;n<rows;n++)
        {
           if(m!=x1 && n!=x2)
           {
           Adj[f1][f2]=arrA[m][n];
           f2++;
           if(f2==k)
           {

               f2=0;
               f1 +=1;
           }
           if(f1==k)
               f1=0;

           }




               }


    Adj_det=matrix_Det(Adj);
    arrA_inv[x1][x2]=Adj_det/Det*pow(-1,x1+x2);
        }


}
    return matrix_Trans(arrA_inv);
}



vector <vector<double> > matrix_sub(vector<vector<double> > arrA, vector<vector<double> > arrB)
{
    //the rows of arrA
    int rowA = arrA.size();
    //the columns of arrA
    int colA = arrA[0].size();
    //the rows of arrB
    int rowB = arrB.size();
    //the columns of arrB
    int colB = arrB[0].size();
    //the result of matrix addition
    vector<vector<double> >  res;
    if (colA != colB || rowA !=rowB)
    {
        cout<<"Illegal Addition!\n";
        return res;
    }
    else
    {
        //Initialization of res
        res.resize(rowA);
        for (int i = 0; i < rowA; ++i)
        {
            res[i].resize(colB);
        }

        //addition
        for (int i = 0; i < rowA; ++i)
        {
            for (int j = 0; j < colA; ++j)
            {
                    res[i][j] += arrA[i][j] -arrB[i][j];

            }
        }
    }
    return res;
}
