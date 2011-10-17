#include "utilFunc.h"
//Mul matrices 4x4 A*B=C
void mulMatrix(float A[][HMATRIX_SIZE], float B[][HMATRIX_SIZE], float C[][HMATRIX_SIZE])
{
	for (int r=0; r < HMATRIX_SIZE ; r++)
		for (int i=0; i < HMATRIX_SIZE; i++)
		{
			C[i][r] = 0;
			for (int j=0; j < HMATRIX_SIZE; j++)
				C[i][r]+= A[i][j]*B[j][r];
		}
	//printf("Check the matrices mul\n");
	//printMatrix(C);	
}
//Inverse of orthonormal matrix
void invOrthMatrix(float tip[][HMATRIX_SIZE], float tip_inv[][HMATRIX_SIZE])
{
	for (int i=0; i<(HMATRIX_SIZE);i++)
	{
		tip_inv[HMATRIX_SIZE-1][i] = tip[HMATRIX_SIZE-1][i];
	}

	for (int i=0; i<(HMATRIX_SIZE-1);i++)
	{
		for (int j=0; j<(HMATRIX_SIZE-1);j++)
		{
			tip_inv[i][j] = tip[j][i];
		}
	}

	float sum;
	for (int i=0; i<HMATRIX_SIZE-1;i++)
	{
		sum = 0;
		for (int j=0; j<HMATRIX_SIZE-1;j++)
		{
			sum+=tip_inv[i][j]*tip[j][HMATRIX_SIZE-1];
		}
		tip_inv[i][HMATRIX_SIZE-1] = -1*sum;
	}
}

//Inverse of orthonormal matrix
void invBigOrthMatrix(float tip[][16], float tip_inv[][16])
{
	for (int i=0; i<(16);i++)
	{
		tip_inv[16-1][i] = tip[16-1][i];
	}

	for (int i=0; i<(16-1);i++)
	{
		for (int j=0; j<(16-1);j++)
		{
			tip_inv[i][j] = tip[j][i];
		}
	}

	float sum;
	for (int i=0; i<16-1;i++)
	{
		sum = 0;
		for (int j=0; j<16-1;j++)
		{
			sum+=tip_inv[i][j]*tip[j][16-1];
		}
		tip_inv[i][16-1] = -1*sum;
	}
}

//Prints a 4x4 matrix
void printMatrix(float matrix[][HMATRIX_SIZE], char *str)
{
	printf("%s\n", str);
	for (int i=0; i<HMATRIX_SIZE; i++) 
	{
		for (int j=0; j<HMATRIX_SIZE; j++)
		{
				printf("%7.3f  ",matrix[i][j]);
		}
		printf("\n");
	}
}

//Prints the result vector
void printVector(float* vector)
{
	if (vector[0] == 0)
	{
		printf("There is no solution to the problem\n");
	} else {
		printf("The solution is: \n");
		for (int i=1; i<INV_VECTOR_SIZE; i++)
		{
			printf("%f ",vector[i]);
		}
	}
	printf("\n");
}

void mulMatrix3x3f(float A[3][3], float B[3][3], float C[3][3])
{
	int r, i, j;
	for (r = 0; r < 3 ; r++)
		for (i = 0; i < 3; i++)
		{
			C[i][r] = 0;
			for (j = 0; j < 3; j++)
				C[i][r] += A[i][j] * B[j][r];
		}
}


void mulMatrix3x3_vector3x1(float A[3][3], float V[3][1], float C[3][1])
{
	for (int r=0; r < 1 ; r++)
		for (int i=0; i < 3; i++)
		{
			C[i][r] = 0;
			for (int j=0; j < 3; j++)
				C[i][r]+= A[i][j]*V[j][r];
		}

}
void printMatrix3x3(float rot_matrix[3][3], char *str)
{
	printf("%s\n", str);
	printf("%5.2f %5.2f %5.2f\n", rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2]);
	printf("%5.2f %5.2f %5.2f\n", rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2]);
	printf("%5.2f %5.2f %5.2f\n", rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2]);
}

void printVector3x1(float v[3][1], char *str)
{
	printf("%s\n", str);
	printf("%5.2f %5.2f %5.2f\n", v[0][0], v[1][0], v[2][0]);
}

void transposeMatrix3x3(float A[3][3], float T[3][3])
{
	for (int r=0; r < 3 ; r++)
		for (int i=0; i < 3; i++)
		{
			T[i][r] = A[r][i];
		}
}


void mulMatrix3x3di(float A[3][3], float B[3][3], float C[3][3])
{
	int r, i, j;
	for (r = 0; r < 3 ; r++)
		for (i = 0; i < 3; i++)
		{
			C[i][r] = 0;
			for (j = 0; j < 3; j++)
				C[i][r] += A[i][j] * B[j][r];
		}
}

unsigned int byteswap (unsigned int nLongNumber)
{
   return (((nLongNumber&0x000000FF)<<24)+((nLongNumber&0x0000FF00)<<8)+
   ((nLongNumber&0x00FF0000)>>8)+((nLongNumber&0xFF000000)>>24));
}



void create_rotation_matrix(float n[3], float phi, float mr[3][3])
{
	float sp = sin(phi);
	float s5p = SQR(sin(0.5f*phi));

	// normalize n
	float n_length = sqrt(SQR(n[0]) + SQR(n[1]) + SQR(n[2]));
	n[0] /= n_length;
	n[1] /= n_length;
	n[2] /= n_length;

	mr[0][0] = 1.0f-2.0f*(SQR(n[1])+SQR(n[2])) * s5p;
	mr[0][1] = -n[2]*sp+2.0f*n[0]*n[1] * s5p;
	mr[0][2] = n[1]*sp+2.0f*n[2]*n[0] * s5p;

	mr[1][0] = n[2]*sp+2.0f*n[0]*n[1] * s5p;
	mr[1][1] = 1.0f-2.0f*(SQR(n[2])+SQR(n[0])) * s5p;
	mr[1][2] = -n[0]*sp+2.0f*n[1]*n[2] * s5p;

	mr[2][0] = -n[1]*sp+2.0f*n[2]*n[0] * s5p;
	mr[2][1] = n[0]*sp+2.0f*n[1]*n[2] * s5p;
	mr[2][2] = 1.0f-2.0f*(SQR(n[0])+SQR(n[1])) * s5p;

}

double distance(double x1, double y1, double x2, double y2){
	return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}

double distance(double x1, double y1, double z1, double x2, double y2, double z2){
	return sqrt(pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2));
}