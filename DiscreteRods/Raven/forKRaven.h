#include "constants.h"
#include "utilFunc.h"
#include "DS0.h"

//declare the functions

//The function gets the angles and distances of the four robots and preforms the forward kinematics
//Input: tx/d4 a vectror in the size of the number of robots. Each element is a the corresponding angle or distance
//of a specific robot. for example t1[2] is the t1 angle of robot number 3.
//Input: The function calculate the position and rotation matrix for each robot and put it in the tip matrix.
//The function fills the tip matrix in the right values. The tip matrix is an array in the size of Numer_of_robots X HMATRIX_SIZE X HMATRIX_SIZE 
//and it contains the position matrix of each robot's tip.
void forK(float* t1,float* t2,float* t3,float* d4,float* t5,float* t6, float tip[][HMATRIX_SIZE][HMATRIX_SIZE]);


//Calculate the transformation matrix according to the dh parameters,
//fill the A matrix with the transformation 
void dh(float theta, float d, float a, float alpha, float A[HMATRIX_SIZE][HMATRIX_SIZE]);

//Moves data from the two dimentional matrix to the four dimentional matrix. The data will be stored at the 
//requested robot and current joint/DOF
void moveTo(float A[NUM_OF_ROBOTS][MAX_DOF_PER_MECH][HMATRIX_SIZE][HMATRIX_SIZE], float B[HMATRIX_SIZE][HMATRIX_SIZE],int robot,int curr_DOF);

//Moves data from the four dimentional matrix to the two dimentional matrix
void moveFrom(float A[NUM_OF_ROBOTS][MAX_DOF_PER_MECH][HMATRIX_SIZE][HMATRIX_SIZE], float B[HMATRIX_SIZE][HMATRIX_SIZE],int robot,int curr_DOF);
