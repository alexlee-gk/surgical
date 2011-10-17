//--------------------------
//All the include functions
//--------------------------
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>    
#include <stdio.h>
#include "obj.h"

//--------------------------
//Enter the defines in here
//--------------------------

struct ModelHolder{
	ObjModel *base, *lower_link, *upper_link, *tool;
};

//------------------------
//Robot Actuall size in mm
//------------------------

#define BASE_WIDTH 100.0
#define BASE_LENGTH 250.0
#define BASE_THICK 25.0

//Length and radius of first cylinder
#define L_CYLINDER_1 46.0
#define R_CYLINDER_1 10.0
#define ACT_LINK_1_DESIGN_DIST_TO_JOINT_AXIS 145
#define LINK1_DESIGN_ANGLE 75.0f

//Length of radius second cylinder
#define L_CYLINDER_2 46.0
#define R_CYLINDER_2 10.0
#define ACT_LINK_2_DESIGN_DIST_TO_JOINT_AXIS 145
#define LINK2_DESIGN_ANGLE 52.0f

//Length and radius of third joint
#define L_JOINT_3 47.0
#define R_JOINT_3 10.0

//Parameters of the fourth joint's base
#define BASE_JOINT4_WIDTH 60.0
#define BASE_JOINT4_LENGTH 96.0
#define BASE_JOINT4_THICK 25.0
#define BASE_JOINT4_DISTANCE 25.0
#define CYLINDER_AXIS 145.0

//Length of radius of fourth joint
#define L_JOINT_4 450.0-BASE_JOINT4_LENGTH
#define R_JOINT_4 3.0

//Length of radius fifth cylinder
#define L_CYLINDER_5 R_JOINT_4*2
#define R_CYLINDER_5 R_JOINT_4

//The parameters of the fifth link
#define BASE_JOINT5_WIDTH 4.0
#define BASE_JOINT5_LENGTH 8.7
#define BASE_JOINT5_THICK 4.0

//Length of radius sixth cylinder
#define L_CYLINDER_6 R_JOINT_4*2
#define R_CYLINDER_6 R_JOINT_4

//parameters tool
#define TOOL_WIDTH 2.0
#define TOOL_LENGTH 20.6
#define TOOL_THICK 2.0
#define TOOL_CLOSING_INTERVAL_1 .05
#define INITIAL_TOOL_ANGLE_1 1.0
#define CLOSED_TOOL_ANGLE_1 -0.5
#define TOOL_CLOSING_INTERVAL_2 .1
#define INITIAL_TOOL_ANGLE_2 -2.8
#define CLOSED_TOOL_ANGLE_2 -1.3

//------------------------
//Robot Sim sizes unitless
//------------------------
#define BASE_TILT_ANGLE -25.0f

#define SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS ACT_LINK_1_DESIGN_DIST_TO_JOINT_AXIS 

#define SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS ACT_LINK_2_DESIGN_DIST_TO_JOINT_AXIS


//General Constants
//where to put the initial position of the objects in the space 
//#define X_INIT 0.0f
//#define Y_INIT 0.0f
//#define Z_INIT 0.0f
//The number of stacks and slices for the spheres and cylinders
#define SLICES 10
#define STACKS 10


//change the orientation of joint 1 to have the same angles as Daneil has.
#define ADJUST_JOINT2_CONV 180.0f

/*This file defines the constants and the function of Creating our Robot.*/


void drawModel(ObjModel *model);
//Recieves the robot current parameters(angles and distances) and draw the robot, in the position that required (xInit, yInit, zInit [mm]).
int DrawObjRobot(float *rtheta, int robot, GLfloat xInit, GLfloat yInit, GLfloat zInit, ModelHolder modelHolder);
int DrawObjRobot(float *rtheta, int robot, GLfloat xInit, GLfloat yInit, GLfloat zInit, int buttonState, ModelHolder modelHolder);

//Draw Base - draws the Robot's base according to the given Dimensions.
//Width along x, length along z and thick along y.
//the box is built under the conditions that: the origin is in the middle of the Width and Thick and in the end of the length. 
//The color can be set by the red, green and blue variables.
int DrawObjBase (GLfloat baseWidth, GLfloat baseLength, GLfloat baseThick);
void DrawObjBase(ObjModel *model, int robot);

//Draw first joint according to the provided angle
int DrawObjFirstJoint(GLfloat rangle1);
void DrawObjFirstJoint(ObjModel *model, int robot, GLfloat rangle1);

//Draw second joint according to the provided angle
int DrawObjSecondJoint(GLfloat rangle2);
void DrawObjSecondJoint(ObjModel *model, int robot, GLfloat rangle2);

//Draw Third joint according to the provided angle
int DrawObjThirdJoint(GLfloat rangle3);

//Draw Fourth joint according to the provided distance
void DrawObjFourthJoint(ObjModel *model, int robot, GLfloat distance4, GLfloat rangle3);

//Draw Fifth joint according to the provided distance
int DrawObjFifthJoint(GLfloat rangle5);

//Draw Sixth joint according to the provided distance
int DrawObjSixthJoint(GLfloat rangle6);

//Draw Tool
int DrawObjTool(GLfloat rangle7, int buttonState, int robot);

void drawObjAxes();

GLdouble (*getObjToolMatrices())[16];
