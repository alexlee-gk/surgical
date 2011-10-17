#include "DrawObjRobot.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "obj.h"
#include "constants.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
using std::ofstream;

float currentObjToolAngle1[4] = {INITIAL_TOOL_ANGLE_1, INITIAL_TOOL_ANGLE_1, INITIAL_TOOL_ANGLE_1, INITIAL_TOOL_ANGLE_1};
float currentObjToolAngle2[4] = {INITIAL_TOOL_ANGLE_2, INITIAL_TOOL_ANGLE_2, INITIAL_TOOL_ANGLE_2, INITIAL_TOOL_ANGLE_2};
GLdouble objToolMatrices[NUM_OF_ROBOTS][16];

void drawModel(ObjModel *model){
	glPushMatrix();
	ObjTriangle *curr;
	int v1, v2, v3, vn1, vn2, vn3;
	glScalef(700, 700, 700);
	glBegin(GL_TRIANGLES);
	for (int i=0; i<model->nTriangle; i++){
		curr = &model->TriangleArray[i];
		v1 = curr->Vertex[0];
		v2 = curr->Vertex[1];
		v3 = curr->Vertex[2];
		vn1 = curr->Normal[0];
		vn2 = curr->Normal[1];
		vn3 = curr->Normal[2];
		glNormal3f(model->NormalArray[vn1-1].x, model->NormalArray[vn1-1].y, model->NormalArray[vn1-1].z);
		glVertex3f(model->VertexArray[v1-1].x, model->VertexArray[v1-1].y, model->VertexArray[v1-1].z);
		glNormal3f(model->NormalArray[vn2-1].x, model->NormalArray[vn2-1].y, model->NormalArray[vn2-1].z);
		glVertex3f(model->VertexArray[v2-1].x, model->VertexArray[v2-1].y, model->VertexArray[v2-1].z);
		glNormal3f(model->NormalArray[vn3-1].x, model->NormalArray[vn3-1].y, model->NormalArray[vn3-1].z);
		glVertex3f(model->VertexArray[v3-1].x, model->VertexArray[v3-1].y, model->VertexArray[v3-1].z);
	}
	glEnd();
	glPopMatrix();
}

int DrawObjRobot(float *rtheta, int robot, GLfloat xInit, GLfloat yInit, GLfloat zInit, ModelHolder modelHolder){
	return DrawObjRobot(rtheta, robot, xInit, yInit, zInit, 0, modelHolder);
}

GLfloat rangle3X, rangle5X, rangle6X, distance4X;

int DrawObjRobot(float *rtheta, int robot,
			  GLfloat xInit, GLfloat yInit, GLfloat zInit, int buttonState, ModelHolder modelHolder) 
{
	glColor3d(1, .9, .9);
	glPushMatrix();
	GLfloat rangle1 = rtheta[0] * 180/M_PI;
	GLfloat rangle2 = rtheta[1] * 180/M_PI;
	GLfloat rangle3 = rtheta[2] * 180/M_PI;
	GLfloat distance4 = -rtheta[3];
	GLfloat rangle5 = rtheta[4] * 180/M_PI;
	GLfloat rangle6 = rtheta[5] * 180/M_PI;
	GLfloat rangle7 = rtheta[6] * 180/M_PI;
	rangle3X = rangle3;
	distance4X = distance4;
	rangle5X = rangle5;
	rangle6X = rangle6;

	glTranslatef(xInit, yInit, zInit);
	if (robot == 2 || robot == 3)			//********
		glRotatef(180, 0, 1, 0);
	glPushMatrix();
		glTranslated(0.0, 0.0, 192);
//		drawObjAxes();
	glPopMatrix();



//	printf("dr: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n", rangle1, rangle2, rangle3, distance4, rangle5, rangle6, rangle7);

	//Draw the base of the robot
	//Tilt the base  
	//glPushMatrix();

	float base_tilt_angle;
	if (robot == 0 || robot == 3)			//********
		base_tilt_angle = BASE_TILT_ANGLE;
	else
		base_tilt_angle = -BASE_TILT_ANGLE;
	
	glRotatef(base_tilt_angle,0.0f,0.0f,1.0f);

	
	
//	DrawObjBase(BASE_WIDTH, BASE_LENGTH,BASE_THICK);
	glPushMatrix();
	if (robot == 0 || robot == 1) glRotatef(base_tilt_angle,0.0f,0.0f,1.0f);
	else glRotatef(-1*base_tilt_angle,0.0f,0.0f,1.0f);
	DrawObjBase(modelHolder.base, robot);
	glPopMatrix();
	//glPopMatrix();

	//glPushMatrix();
	//Draw First Joint and tilt it as required
	glRotatef(-(90.0f+base_tilt_angle),0.0f,0.0f,1.0f);




	glTranslatef(0.0f,0.0f,L_CYLINDER_1/2);
	if (robot == 0 || robot == 3) {
		DrawObjFirstJoint(modelHolder.lower_link, robot, rangle1);
//		glRotatef(rangle1,0.0f,0.0f,1.0f);
	} else {
		DrawObjFirstJoint(modelHolder.lower_link, robot, 180.0-rangle1);
//		glRotatef(180.0-rangle1,0.0f,0.0f,1.0f);
	}
	//glPopMatrix();


	
	//glPushMatrix();
	//move the origin and the orientation for the second joint.
    glTranslatef(0.0f,(SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1)*cos(M_PI*(90-LINK1_DESIGN_ANGLE)/180),
			(SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1/2)-(SIM_LINK1_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_1)
			*sin(M_PI*(90-LINK1_DESIGN_ANGLE)/180));
	glRotatef(LINK1_DESIGN_ANGLE,1.0f,0.0f,0.0f);




	if (robot == 0 || robot == 3){			//********
		DrawObjSecondJoint(modelHolder.upper_link, robot, rangle2);
//		glRotatef(rangle2,0.0f,0.0f,1.0f);
	}else{
		DrawObjSecondJoint(modelHolder.upper_link, robot, -rangle2);
//		glRotatef(-rangle2,0.0f,0.0f,1.0f);
	}
	//move the origin and the orientation for the third joint.
	glTranslatef(0.0f,(SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2)*cos(M_PI*(90-LINK2_DESIGN_ANGLE)/180),
			(SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2/2)-(SIM_LINK2_DESIGN_DIST_TO_JOINT_AXIS+L_CYLINDER_2)
			*sin(M_PI*(90-LINK2_DESIGN_ANGLE)/180));
	glRotatef(LINK2_DESIGN_ANGLE,1.0f,0.0f,0.0f);
	
	
	if (robot == 0)
		glTranslatef(-39, -30, 0);
	else if (robot == 1)
		glTranslatef(54, -30, 0);
	else if (robot == 2)
		glTranslatef(60, -40, 0);
	else if (robot == 3)
		glTranslatef(-48, -26, 0);


	//Draw the fourth joint
	if (robot == 0 || robot == 3)			//********
		DrawObjFourthJoint(modelHolder.tool, robot, distance4, rangle3);
	else
		DrawObjFourthJoint(modelHolder.tool, robot, distance4, -rangle3);
	
	

	//Draw tool
	DrawObjTool(rangle7, buttonState, robot);

	glPopMatrix();
	return true;
}
int DrawObjTool(GLfloat rangle7, int buttonState, int robot)
{
	//rotate to correct orientation (same as long tool arm)
	glRotatef(344, 1, 0, 0);

	if (robot == 0 || robot == 3)
		glRotatef(rangle3X,0.0f,0.0f,1.0f);
	else
		glRotatef(-rangle3X,0.0f,0.0f,1.0f);
	
	glTranslatef(0, 0, distance4X+120);

	glRotatef(90.0f,0.0f,1.0f,0.0f);
	glRotatef(90.0f,0.0f,0.0f,1.0f);

	glRotatef(-rangle5X,0.0f,0.0f,1.0f);
	glRotatef(-90.0f,1.0f,0.0f,0.0f);

	glRotatef(90.0f,0.0f,1.0f,0.0f);
	if (robot == 0 || robot == 3)			//********
		glRotatef(rangle6X,0.0f,0.0f,1.0f);
	else
		glRotatef(-rangle6X,0.0f,0.0f,1.0f);





	glRotatef(-90.0f,0.0f,1.0f,0.0f);
	if (buttonState == 0){
		if (currentObjToolAngle1[robot] < INITIAL_TOOL_ANGLE_1)
			currentObjToolAngle1[robot] += TOOL_CLOSING_INTERVAL_1;
	} else {
		if (currentObjToolAngle1[robot] > CLOSED_TOOL_ANGLE_1){
			currentObjToolAngle1[robot] -= TOOL_CLOSING_INTERVAL_1;
		}
	}
	
	glRotatef(currentObjToolAngle1[robot]*rangle7, 1.0f, 0.0f, 0.0f);
	glTranslatef(0.0f,0.0f,TOOL_LENGTH);
	//Switch between thick and width since the axis are different
	DrawObjBase(TOOL_THICK,TOOL_LENGTH,TOOL_WIDTH);
	glTranslatef(0.0f,0.0f,-TOOL_LENGTH);
	
	if (buttonState == 0){
		if (currentObjToolAngle2[robot] > INITIAL_TOOL_ANGLE_2)
			currentObjToolAngle2[robot] -= TOOL_CLOSING_INTERVAL_2;
	} else {
		if (currentObjToolAngle2[robot] < CLOSED_TOOL_ANGLE_2)
			currentObjToolAngle2[robot] += TOOL_CLOSING_INTERVAL_2;	
	}
	glRotatef(currentObjToolAngle2[robot]*rangle7, 1.0f, 0.0f, 0.0f);
	glTranslatef(0.0f,0.0f,TOOL_LENGTH);
	//Switch between thick and width since the axis are different
	DrawObjBase(TOOL_THICK,TOOL_LENGTH,TOOL_WIDTH);
	
	return true;
}

void DrawObjFourthJoint(ObjModel *model, int robot, GLfloat distance4, GLfloat rangle3){
	glPushMatrix();

	//rotate to correct orientation
	glRotatef(344, 1, 0, 0);
	if (robot == 1 || robot == 2)
		glRotatef(180, 0, 0, 1);
			
	//move distance

	glRotatef(rangle3,0.0f,0.0f,1.0f);

	glTranslatef(0, 0, distance4+120);
	//move to origin
	if (robot == 1 || robot == 3)
		glRotatef(105.9, 1, 0, 0);
	glRotatef(37.1, 1, 0, 0);
	glRotatef(90, 0, 0, 1);
	if (robot == 1 || robot == 3)
		glTranslatef(-1, 7, -110);
	
	glColor3f(1, .9, .9);
	glTranslatef(335, 151, 100);
	drawModel(model);
	glPopMatrix();
}

void DrawObjSecondJoint(ObjModel *model, int robot, GLfloat rangle2){
	glPushMatrix();

	if (robot == 1 || robot == 2)
		glTranslatef(10, 0, 0);
	glTranslatef(-10, -55, 55);
	
	//adjust angles to Daniel convention
	glRotatef(rangle2,0.0f,0.0f,1.0f);

	//move model to the origin
	if (robot == 2 || robot ==3)
		glRotatef(180, 1, 0, 0);
	if (robot == 1 || robot == 2)
		glRotatef(180, 0, 1, 0);
	if (robot == 0 || robot == 1)
		glRotatef(270, 0, 0, 1);
	else
		glRotatef (90, 0, 0, 1);

	if (robot == 1 || robot == 3)
		glTranslatef(0, 0, -350);
	glTranslatef(340, 160, 220);
	drawModel(model);
	glPopMatrix();


	glRotatef(rangle2,0.0f,0.0f,1.0f);
}

void DrawObjFirstJoint(ObjModel *model, int robot, GLfloat rangle1){

	//move the origin to the middle of the joint
	glTranslatef(0.0f,0.0f,L_CYLINDER_1/2);
	glRotatef(rangle1,0.0f,0.0f,1.0f);

	glTranslatef(0, 0, -20);
	glPushMatrix();
	//move model to the origin
	glRotatef(90, 1, 0, 0);
	glRotatef(270, 0, 0, 1);
	if (robot == 1 || robot == 3)
		glRotatef(180, 1, 0, 0);
	glTranslatef(258, 155, 103);
	if (robot == 1 || robot == 3)
		glTranslatef(0, 0, -117);

	drawModel(model);
	glPopMatrix();
}

int DrawObjBase(GLfloat baseWidth, GLfloat baseLength, GLfloat baseThick)
{			
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

void DrawObjBase(ObjModel *model, int robot){
	glPushMatrix();
	if (robot == 1 || robot == 3)
		glTranslatef(-80, 0, 0);
	if (robot == 3)
		glTranslatef(-25, 10, 0);
	else if (robot == 2)
		glTranslatef(25, 10, 0);
	glTranslatef(90, 170, -230);
	glRotatef(90, 0, 1, 0);
//	glRotatef(90, 0, 1, 0);
	drawModel(model);
	glPopMatrix();
}

void drawObjAxes()
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

GLdouble (*getObjToolMatrices())[16]{
	return objToolMatrices;
}
