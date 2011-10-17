
#include "forKRaven.h"

//The code is an exact translation from Daneil's matlab code
void forK(float* t1, float* t2, float* t3, float* d4, float* t5, float* t6, float tip[][HMATRIX_SIZE][HMATRIX_SIZE])
{
/*
	t1[0] = 0.0 	*M_PI/180.0;
	t2[0] = 90.0 	*M_PI/180.0;
	t3[0] = 30.0 	*M_PI/180.0;
	d4[0] = -100.0;
	t5[0] = 30.0 	*M_PI/180.0;
	t6[0] = 30.0 	*M_PI/180.0;
//	t7[0] = -30.0 	*M_PI/180.0;
*/

	float theta[NUM_OF_ROBOTS][MAX_DOF_PER_MECH];
	theta[0][0] = t1[0]; 	theta[0][1] = -t2[0]; 	theta[0][2] = M_PI/2-t3[0]; 	theta[0][3] = 0;
	theta[0][4] = M_PI/2-t5[0]; 	theta[0][5] = M_PI/2+t6[0]; 	theta[0][6] = 0; 

	theta[1][0] = M_PI-t1[1]; 	theta[1][1] = t2[1];	theta[1][2] = M_PI+M_PI/2+t3[1];	theta[1][3] = 0;
	theta[1][4] = M_PI/2+t5[1];	theta[1][5] = M_PI/2-t6[1];	theta[1][6] = 0;

	theta[2][0] = t1[2];	theta[2][1] = -t2[2];	theta[2][2] = t3[2];	theta[2][3] = 0;
	theta[2][4] = t5[2];	theta[2][5] = t6[2];	theta[2][6] = 0;
	 
	theta[3][0] = M_PI-t1[3];	theta[3][1] = t2[3];	theta[3][2] = t3[3];	theta[3][3] = 0;
	theta[3][4] = t5[3];	theta[3][5] = t6[3];	theta[3][6] = 0;

	float d[NUM_OF_ROBOTS][MAX_DOF_PER_MECH];
	for (int i = 0; i < NUM_OF_ROBOTS; i++)
	{
		for (int j = 0; j<MAX_DOF_PER_MECH; j++)
		{
			if (j == PLACE_DIST_IN_POSITION_VECTOR)
			{
				d[i][j] = d4[i];
			} else {
				d[i][j] = 0;
			}
		}
	}
	//It will be easier to the matlab code reader to understand.
	float a5 = VALUEA5;
	float a[MAX_DOF_PER_MECH];
	a[0]=0;a[1]=0;a[2]=0;a[3]=0;a[4]=a5;a[5]=0;a[6]=1;

	float a1 = DESIGN_ANGLE1;
	float b1 = DESIGN_ANGLE2;

	float alpha[NUM_OF_ROBOTS][MAX_DOF_PER_MECH];
	alpha[0][0] = M_PI-a1; 	alpha[0][1] = -b1; 		alpha[0][2] = 0; 	alpha[0][3] = -M_PI/2;
	alpha[0][4] = M_PI/2; 	alpha[0][5] = -M_PI/2; 	alpha[0][6] = 0; 

	alpha[1][0] = M_PI-a1; 	alpha[1][1] = -b1; 		alpha[1][2] = 0; 	alpha[1][3] = -M_PI/2;
	alpha[1][4] = -M_PI/2; 	alpha[1][5] = -M_PI/2; 	alpha[1][6] = 0;

	alpha[2][0] = M_PI-a1; 	alpha[2][1] = -b1; 	alpha[2][2] = 0; 	alpha[2][3] = -M_PI/2;
	alpha[2][4] = M_PI/2; 	alpha[2][5] = 0; 	alpha[2][6] = 0;

	alpha[3][0] = M_PI-a1; 	alpha[3][1] = -b1; 	alpha[3][2] = 0; 	alpha[3][3] = -M_PI/2;
	alpha[3][4] = M_PI/2; 	alpha[3][5] = 0; 	alpha[3][6] = 0;

	float T[NUM_OF_ROBOTS][MAX_DOF_PER_MECH][HMATRIX_SIZE][HMATRIX_SIZE];
	float temp1[HMATRIX_SIZE][HMATRIX_SIZE];
	float temp2[HMATRIX_SIZE][HMATRIX_SIZE];
	float temp3[HMATRIX_SIZE][HMATRIX_SIZE];

	//float Tx[NUM_OF_ROBOTS][MAX_DOF_PER_MECH][HMATRIX_SIZE][HMATRIX_SIZE];
	//going according to Daniel matlab program
	for (int r = 0; r < NUM_OF_ROBOTS; r++)
	{
		dh(theta[r][0], d[r][0], a[0], alpha[r][0], temp1);
		moveTo(T,temp1,r,0);
		//moveTo(Tx,temp1,r,0);
		//moveFrom(Tx,temp1,r,0);
		//printMatrix(temp1);
	}

	
	for (int i = 1; i < MAX_DOF_PER_MECH; i++)
	{
		for (int r = 0; r < NUM_OF_ROBOTS ; r++)
		{
			//T(:, :, i, r) = T(:, :, i-1, r) * dh(theta(r, i), d(r, i), a(i), alpha(r, i));
			dh(theta[r][i], d[r][i], a[i], alpha[r][i], temp1);
			//printf("the dh params\n");
			//printMatrix(temp1);	
			moveFrom(T,temp2,r, i-1);
			//printf("the T(:,:,i-1,r)\n");
			//printMatrix(temp2);			
			mulMatrix(temp2, temp1, temp3);
			//printf("the T(:,:,i,r)\n");
			//printMatrix(temp3);
			moveTo(T,temp3,r,i);
			//moveFrom(T,temp3,r,i);
			//printMatrix(temp3);
			//Tx(:, :, i, r) = dh(theta(r, i), d(r, i), a(i), alpha(r, i));
			//moveTo(T,temp1,r,i);
		}
	}

	for (int r = 0; r < NUM_OF_ROBOTS ; r++)
	{
		for (int i = 0; i < HMATRIX_SIZE; i++)
		{
			for (int j = 0; j < HMATRIX_SIZE; j++)
			{
				tip[r][i][j] = T[r][MAX_DOF_PER_MECH-2][i][j];
			}
		}
	}


	printf("%s\n", "test1");
	printf("%7.4f %7.4f %7.4f %7.4f\n", tip[0][0][0], tip[0][0][1], tip[0][0][2], tip[0][0][3]);
	printf("%7.4f %7.4f %7.4f %7.4f\n", tip[0][1][0], tip[0][1][1], tip[0][1][2], tip[0][1][3]);
	printf("%7.4f %7.4f %7.4f %7.4f\n", tip[0][2][0], tip[0][2][1], tip[0][2][2], tip[0][2][3]);
	printf("%7.4f %7.4f %7.4f %7.4f\n", tip[0][3][0], tip[0][3][1], tip[0][3][2], tip[0][3][3]);

}
//Moves data from the two dimentional matrix to the four dimentional matrix. The data will be stored at the 
//requested robot and current joint/DOF
void moveTo(float A[NUM_OF_ROBOTS][MAX_DOF_PER_MECH][HMATRIX_SIZE][HMATRIX_SIZE], float B[HMATRIX_SIZE][HMATRIX_SIZE],int robot,int curr_DOF)
{
	for (int i = 0; i < HMATRIX_SIZE; i++)
	{
		for (int j = 0; j < HMATRIX_SIZE; j++)
		{
			A[robot][curr_DOF][i][j] = B[i][j];
		}
	}
}
//Moves data from the four dimentional matrix to the two dimentional matrix
void moveFrom(float A[NUM_OF_ROBOTS][MAX_DOF_PER_MECH][HMATRIX_SIZE][HMATRIX_SIZE], float B[HMATRIX_SIZE][HMATRIX_SIZE],int robot,int curr_DOF)
{
	for (int i = 0; i < HMATRIX_SIZE; i++)
	{
		for (int j = 0; j < HMATRIX_SIZE; j++)
		{
			B[i][j] = A[robot][curr_DOF][i][j];
		}
	}
}
void dh(float theta, float d, float a, float alpha, float A[HMATRIX_SIZE][HMATRIX_SIZE]){

	A[0][0] = cos(theta);
	A[0][1] = -cos(alpha)*sin(theta);
	A[0][2] = sin(alpha)*sin(theta);
	A[0][3] = a*cos(theta);

	A[1][0] = sin(theta);
	A[1][1] = cos(alpha)*cos(theta);
	A[1][2] = -sin(alpha)*cos(theta);
	A[1][3] = a*sin(theta);

	A[2][0] = 0;
	A[2][1] = sin(alpha);
	A[2][2] = cos(alpha);
	A[2][3] = d;

	A[3][0] = 0;
	A[3][1] = 0;
	A[3][2] = 0;
	A[3][3] = 1;
}
