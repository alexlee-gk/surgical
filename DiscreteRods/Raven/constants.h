#ifndef CON_KIN
#define CON_KIN
//Put all the includes here

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
//for some reason abs is in stdlib
//#include <stdlib.h>

//Declare some global variables and constants here

#define HMATRIX_SIZE 4
#define INV_VECTOR_SIZE 7
#define EPS2 0.00001
#define EPS1 0.001
#define PI 3.1415926535
#define NUM_OF_ROBOTS 4
#define DESIGN_ANGLE1 75*PI/180;
#define DESIGN_ANGLE2 52*PI/180;
#define VALUEA5 8.7
//Include the tool open and close angle

#define PLACE_DIST_IN_POSITION_VECTOR 3
#define SQ(x) ((x)*(x))
#define MAX_LINE_SIZE 30
#endif
