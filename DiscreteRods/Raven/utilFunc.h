#ifndef UTLFUNC
#define UTLFUNC
#include "constants.h"
#endif

#define SQR(x) ((x) * (x))

//Inverse the position matrix in a fast way
//Assumptions: The matrix is orthogonal position and orientation matrix
//Input: tip, a pointer to a position and orientation matrix
// a pointer to the inverse position and orientation matrix
void invOrthMatrix(float tip[][HMATRIX_SIZE], float tip_inv[][HMATRIX_SIZE]);
void invBigOrthMatrix(float tip[][16], float tip_inv[][16]);

//Print a 4x4 matrix
//input a pointer to the matrix
void printMatrix(float matrix[][HMATRIX_SIZE], char *str);

//Print the result vector in the size of INV_VECTOR_SIZE
void printVector(float* vector); 

//multiple two 4x4 matrix
//Input: A and B pointers to the 4x4 matrix. 
//The function fills the result matrix C 
void mulMatrix(float A[][HMATRIX_SIZE], float B[][HMATRIX_SIZE], float C[][HMATRIX_SIZE]);


void mulMatrix3x3f(float A[3][3], float B[3][3], float C[3][3]);

void mulMatrix3x3_vector3x1(float A[3][3], float V[3][1], float C[3][1]);
void printMatrix3x3(float [3][3], char *str);
void printVector3x1(float [3][1], char *str);

void transposeMatrix3x3(float A[3][3], float T[3][3]);


unsigned int byteswap (unsigned int nLongNumber);
void mulMatrix3x3di(float A[3][3], float B[3][3], float C[3][3]);

void create_rotation_matrix(float n[3], float phi, float mr[3][3]);

double distance(double x1, double y1, double x2, double y2);
double distance(double x1, double y1, double z1, double x2, double y2, double z2);