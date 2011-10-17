#include "DrawRobot.h"
#include "forKRaven.h"
#define _USE_MATH_DEFINES
#include <math.h>

float currentToolAngle1[4] = {INITIAL_TOOL_ANGLE_1, INITIAL_TOOL_ANGLE_1, INITIAL_TOOL_ANGLE_1, INITIAL_TOOL_ANGLE_1};
float currentToolAngle2[4] = {INITIAL_TOOL_ANGLE_2, INITIAL_TOOL_ANGLE_2, INITIAL_TOOL_ANGLE_2, INITIAL_TOOL_ANGLE_2};
GLdouble toolMatrices[NUM_OF_ROBOTS][16];

int DrawRobot(float *rtheta, int robot, GLfloat xInit, GLfloat yInit, GLfloat zInit){
	return DrawRobot(rtheta, robot, xInit, yInit, zInit, 0);
}

int DrawRobot(float *rtheta, int robot,
			  GLfloat xInit, GLfloat yInit, GLfloat zInit, int buttonState) 
{
	GLfloat rangle1 = rtheta[0] * 180/M_PI;
	GLfloat rangle2 = rtheta[1] * 180/M_PI;
	GLfloat rangle3 = rtheta[2] * 180/M_PI;
	GLfloat distance4 = -rtheta[3];
	GLfloat rangle5 = rtheta[4] * 180/M_PI;
	GLfloat rangle6 = rtheta[5] * 180/M_PI;
	GLfloat rangle7 = rtheta[6] * 180/M_PI;

	glTranslatef(xInit, yInit, zInit);
	if (robot == 2 || robot == 3)			//********
		glRotatef(180, 0, 1, 0);
	glPushMatrix();
		glTranslated(0.0, 0.0, 192);
		drawAxes();
	glPopMatrix();



	//printf("dr: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n", rangle1, rangle2, rangle3, distance4, rangle5, rangle6, rangle7);

	//Draw the base of the robot
	//Tilt the base  
	//glPushMatrix();

	float base_tilt_angle;
	if (robot == 0 || robot == 3)			//********
		base_tilt_angle = BASE_TILT_ANGLE;
	else
		base_tilt_angle = -BASE_TILT_ANGLE;

	glRotatef(base_tilt_angle,0.0f,0.0f,1.0f);


	DrawBase(BASE_WIDTH, BASE_LENGTH,BASE_THICK, 0.0f, 0.0f, 0.7f);
	//glPopMatrix();
	
	//glPushMatrix();
	//Draw First Joint and tilt it as required
	glRotatef(-(90.0f+base_tilt_angle),0.0f,0.0f,1.0f);

	if (robot == 0 || robot == 3)			//********
		DrawFirstJoint(rangle1);
	else
		DrawFirstJoint(180.0-rangle1);
	//glPopMatrix();

	//glPushMatrix();
	//move the origin and the orientation for the second joint.
    glTranslatef(0.0f,(SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1)*cos(M_PI*(90-LINK1_DESIGN_ANGLE)/180),
			(SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1/2)-(SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1)
			*sin(M_PI*(90-LINK1_DESIGN_ANGLE)/180));
	glRotatef(LINK1_DESIGN_ANGLE,1.0f,0.0f,0.0f);


	if (robot == 0 || robot == 3)			//********
		DrawSecondJoint(rangle2);
	else
		DrawSecondJoint(-rangle2);
	

	
	//move the origin and the orientation for the third joint.
	glTranslatef(0.0f,(SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2)*cos(M_PI*(90-LINK2_DESIGN_ANGLE)/180),
			(SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2/2)-(SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2)
			*sin(M_PI*(90-LINK2_DESIGN_ANGLE)/180));
	glRotatef(LINK2_DESIGN_ANGLE,1.0f,0.0f,0.0f);
	if (robot == 0 || robot == 3)			//********
		DrawThirdJoint(rangle3);	
	else
		DrawThirdJoint(-rangle3);	
	
	//Draw the fourth joint
	DrawFourthJoint(distance4);	
	
	//move the origin and the orientation for the fifth joint.
	glTranslatef(-R_JOINT_4, 0.0f, distance4);
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glRotatef(90.0f,0.0f,0.0f,1.0f);

	//Draw the fifth joint and link	
	DrawFifthJoint(rangle5);

	//Draw the sixth joint and link
	if (robot == 0 || robot == 3)			//********
		DrawSixthJoint(rangle6);
	else
		DrawSixthJoint(-rangle6);

	//Draw tool
	DrawTool(rangle7, buttonState, robot);

	return true;
}

int DrawTool(GLfloat rangle7, int buttonState, int robot)
{
	glRotatef(-90.0f,0.0f,1.0f,0.0f);
	if (buttonState == 0){
		if (currentToolAngle1[robot] < INITIAL_TOOL_ANGLE_1)
			currentToolAngle1[robot] += TOOL_CLOSING_INTERVAL_1;
	} else {
		if (currentToolAngle1[robot] > CLOSED_TOOL_ANGLE_1){
			currentToolAngle1[robot] -= TOOL_CLOSING_INTERVAL_1;
		}
	}
	
	glRotatef(currentToolAngle1[robot]*rangle7, 1.0f, 0.0f, 0.0f);
	glTranslatef(0.0f,0.0f,TOOL_LENGTH);
	glGetDoublev(GL_MODELVIEW_MATRIX, toolMatrices[robot]);
	//Switch between thick and width since the axis are different
	DrawBase(TOOL_THICK,TOOL_LENGTH,TOOL_WIDTH, 0.5f, 1.0f, 0.0f);

	glTranslatef(0.0f,0.0f,-TOOL_LENGTH);
	
	if (buttonState == 0){
		if (currentToolAngle2[robot] > INITIAL_TOOL_ANGLE_2)
			currentToolAngle2[robot] -= TOOL_CLOSING_INTERVAL_2;
	} else {
		if (currentToolAngle2[robot] < CLOSED_TOOL_ANGLE_2)
			currentToolAngle2[robot] += TOOL_CLOSING_INTERVAL_2;	
	}
	glRotatef(currentToolAngle2[robot]*rangle7, 1.0f, 0.0f, 0.0f);
	glTranslatef(0.0f,0.0f,TOOL_LENGTH);
	//Switch between thick and width since the axis are different
	DrawBase(TOOL_THICK,TOOL_LENGTH,TOOL_WIDTH, 0.5f, 1.0f, 0.5f);
	
	return true;
}

int DrawSixthJoint(GLfloat rangle6)
{
	//move the origin and the orientation for the fifth joint.
	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glTranslatef(0.0f,0.0f,-L_CYLINDER_6/2);
	//draw the cylinder
	glRotatef(rangle6,0.0f,0.0f,1.0f);
	GLUquadric *cylinder6;
	cylinder6 = gluNewQuadric();
	glColor3f(1.0f,0.0f,0.0f);
	gluCylinder(cylinder6,R_CYLINDER_6,R_CYLINDER_6,L_CYLINDER_6*2 ,SLICES,STACKS);
	//return to the middle of the cylinder
	glTranslatef(0.0f,0.0f,L_CYLINDER_6/2);


	return true;
}

int DrawFifthJoint(GLfloat rangle5)
{	
	
	//Draw the cylinder
	GLUquadric *cylinder5;
	cylinder5 = gluNewQuadric();
	glColor3f(1.0f,0.0f,0.0f);
	gluCylinder(cylinder5,R_CYLINDER_5,R_CYLINDER_5,L_CYLINDER_5 ,SLICES,STACKS);

	//Create the fifth link
	glRotatef(-rangle5,0.0f,0.0f,1.0f);
	glTranslatef(0.0f,0.0f,L_CYLINDER_5/2);
	glRotatef(-90.0f,1.0f,0.0f,0.0f);
	glTranslatef(0.0f,0.0f,BASE_JOINT5_LENGTH);
	//Switch between thick and width since the axis are different
	DrawBase(BASE_JOINT5_THICK,BASE_JOINT5_LENGTH,BASE_JOINT5_WIDTH, 0.5f, 0.6f, 0.7f);


	return true;
}

int DrawFourthJoint(GLfloat distance4)
{
	glTranslatef(0.0f,0.0f,-BASE_JOINT4_DISTANCE);
	DrawBase(BASE_JOINT4_WIDTH, BASE_JOINT4_LENGTH,BASE_JOINT4_THICK, 1.0f, 0.0f, 1.0f);

	//set link color to red
	glColor3f(0.5f,0.0f,0.5f);
	//build the cylinder of the first joint
	GLUquadric *cylinder3;
	cylinder3 = gluNewQuadric();		
	gluCylinder(cylinder3,R_JOINT_4, R_JOINT_4, CYLINDER_AXIS+L_JOINT_3+BASE_JOINT4_DISTANCE,SLICES,STACKS);
	//get to the place where distance4 starts
	glTranslatef(0.0f,0.0f,CYLINDER_AXIS+L_JOINT_3+BASE_JOINT4_DISTANCE);

	glColor3f(1.0f,1.0f,0.0f);
	//build the distance4
	GLUquadric *cylinder4;
	cylinder4 = gluNewQuadric();
	gluCylinder(cylinder4,R_JOINT_4,R_JOINT_4,distance4 ,SLICES,STACKS);

	return true;
}

int DrawThirdJoint(GLfloat rangle3)
{
	glColor3f(1.0f,0.0f,0.0f);
	//build the cylinder of the first joint
	GLUquadric *cylinder2;
	cylinder2 = gluNewQuadric();		
	gluCylinder(cylinder2,R_JOINT_3, R_JOINT_3, L_JOINT_3,SLICES,STACKS);
	glRotatef(rangle3,0.0f,0.0f,1.0f);
	return true;
}



int DrawSecondJoint(GLfloat rangle2)
{
	//set link color to red
	glColor3f(1.0f,0.0f,0.0f);
	//build the cylinder of the first joint
	GLUquadric *cylinder2;
	cylinder2 = gluNewQuadric();		
	gluCylinder(cylinder2,R_CYLINDER_2, R_CYLINDER_2, L_CYLINDER_2,SLICES,STACKS);
	glColor3f(0.0f,1.0f,1.0f);

	gluCylinder(gluNewQuadric(), 20, 20, 20, 20, 20);

	//move the origin to the middle of the joint
	glTranslatef(0.0f,0.0f,L_CYLINDER_2/2);
	//adjust angles to Daniel convention
	glRotatef(rangle2,0.0f,0.0f,1.0f);
	//draw the polygon of the first link
    glBegin(GL_POLYGON);							    // Draw A Quad		
		glVertex3f(0.0f,0.0f, -L_CYLINDER_2/2);					
		glVertex3f(0.0f, 0.0f, L_CYLINDER_2/2);
		glVertex3f(0.0f, SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS*cos(M_PI*(90-LINK2_DESIGN_ANGLE)/180), 
			(SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2/2)-SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS*sin(M_PI*(90-LINK2_DESIGN_ANGLE)/180));		
		glVertex3f(0.0f, (SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2)*cos(M_PI*(90-LINK2_DESIGN_ANGLE)/180),
			(SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2/2)-(SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2)
			*sin(M_PI*(90-LINK2_DESIGN_ANGLE)/180));				
    glEnd();

	return true;
}


int DrawFirstJoint(GLfloat rangle1)
{
	//set link color to red
	glColor3f(1.0f,0.0f,0.0f);
	//build the cylinder of the first joint
	GLUquadric *cylinder1;
	cylinder1 = gluNewQuadric();		
	gluCylinder( cylinder1,R_CYLINDER_1, R_CYLINDER_1, L_CYLINDER_1,SLICES,STACKS);
	glColor3f(0.0f,1.0f,0.0f);
	//move the origin to the middle of the joint
	glTranslatef(0.0f,0.0f,L_CYLINDER_1/2);
	glRotatef(rangle1,0.0f,0.0f,1.0f);
	//draw the polygon of the first link
    glBegin(GL_POLYGON);							    // Draw A Quad		
		glVertex3f(0.0f,0.0f, -L_CYLINDER_1/2);					
		glVertex3f(0.0f, 0.0f, L_CYLINDER_1/2);				
		glVertex3f(0.0f, SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS*cos(M_PI*(90-LINK1_DESIGN_ANGLE)/180), 
			(SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1/2)-SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS*sin(M_PI*(90-LINK1_DESIGN_ANGLE)/180));		
		glVertex3f(0.0f, (SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1)*cos(M_PI*(90-LINK1_DESIGN_ANGLE)/180),
			(SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1/2)-(SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1)
			*sin(M_PI*(90-LINK1_DESIGN_ANGLE)/180));				
    glEnd();
	
	return true;
}


int DrawBase(GLfloat baseWidth, GLfloat baseLength, GLfloat baseThick, GLfloat red, GLfloat green, GLfloat blue)
{			
	//set base color to blue
	glColor3f(red,green,blue);
	glNormal3f (0.0f,0.0f,1.0f);
	glBegin(GL_POLYGON);							    // Draw A Quad		
		glVertex3f(-baseWidth/2,baseThick/2, 0.0f);					
		glVertex3f(-baseWidth/2, -baseThick/2, 0.0f);				
		glVertex3f(baseWidth/2, -baseThick/2, 0.0f);		
		glVertex3f(baseWidth/2,baseThick/2, 0.0f);				
    glEnd();
    
	glNormal3f (-1.0f,0.0f,0.0f);
	glBegin(GL_POLYGON);								// Draw A Quad
		glVertex3f(-baseWidth/2,baseThick/2, 0.0f);					
		glVertex3f(-baseWidth/2, baseThick/2, -baseLength);				
		glVertex3f(-baseWidth/2, -baseThick/2, -baseLength);		
		glVertex3f(-baseWidth/2,-baseThick/2, 0.0f);						
    glEnd();
	
	glNormal3f (0.0f,0.0f,-1.0f);
	glBegin(GL_POLYGON);								// Draw A Quad
		glVertex3f(-baseWidth/2, baseThick/2, -baseLength);					
		glVertex3f(baseWidth/2, baseThick/2, -baseLength);				
		glVertex3f(baseWidth/2, -baseThick/2, -baseLength);		
		glVertex3f(-baseWidth/2, -baseThick/2, -baseLength);						
    glEnd();
	
	glNormal3f (1.0f,0.0f,0.0f);
	glBegin(GL_POLYGON);								// Draw A Quad
		glVertex3f(baseWidth/2, baseThick/2, -baseLength);					
		glVertex3f(baseWidth/2, baseThick/2, 0);				
		glVertex3f(baseWidth/2, -baseThick/2, 0);		
		glVertex3f(baseWidth/2, -baseThick/2, -baseLength);						
    glEnd();
	
	glNormal3f (0.0f,1.0f,0.0f);
	glBegin(GL_POLYGON);								// Draw A Quad
		glVertex3f(baseWidth/2, baseThick/2, -baseLength);					
		glVertex3f(-baseWidth/2, baseThick/2, -baseLength);				
		glVertex3f(-baseWidth/2, baseThick/2, 0);		
		glVertex3f(baseWidth/2, baseThick/2, 0);						
    glEnd();
	
	glNormal3f (0.0f,-1.0f,0.0f);
	glBegin(GL_POLYGON);								// Draw A Quad
		glVertex3f(-baseWidth/2, -baseThick/2, -baseLength);					
		glVertex3f(baseWidth/2, -baseThick/2, -baseLength);				
		glVertex3f(baseWidth/2, -baseThick/2, 0);		
		glVertex3f(-baseWidth/2, -baseThick/2, 0);						
    glEnd();

	return true;
}

void drawAxes()
{
	GLUquadric *cylinderX;
	GLUquadric *cylinderY;
	GLUquadric *cylinderZ;

	glColor3f(0.0, 0.0, 1.0);
	//build the cylinder of the first joint
	cylinderX = gluNewQuadric();		
	gluCylinder(cylinderX, 1, 1, 100, SLICES, STACKS);

	glRotated(-90, 1, 0, 0);
	glColor3f(1.0f,0.0f,0.0f);
	//build the cylinder of the first joint
	cylinderY = gluNewQuadric();		
	gluCylinder(cylinderY, 1, 1, 100, SLICES, STACKS);

	glRotated(-90, 0, 1, 0);
	glColor3f(0.0f, 1, 0.0f);
	//build the cylinder of the first joint
	cylinderY = gluNewQuadric();		
	gluCylinder(cylinderY, 1, 1, 100, SLICES, STACKS);
}

GLdouble (*getToolMatrices())[16]{
	return toolMatrices;
}
