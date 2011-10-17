
#include "constants.h"
#include "utilFunc.h"

//declare the functions

//The function gets the position and rotation matrix and preforms the inverse kinematics
//Input: A pointer to the position and rotation matrix (4x4 matrix), the robot number (1 out of four) 
//A pointer to an array of 7 components. in the zero location we put 1 if there is a solution and 0 if not.
//all the other parameters are the raven angles and distances.
int invK(float tip[][HMATRIX_SIZE], float *resultVector, int robotNum); 


