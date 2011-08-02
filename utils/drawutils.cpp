#include "drawutils.h"

void drawCylinder(Vector3d pos, Matrix3d rot, double h, double r, float color0, float color1, float color2) {
	glPushMatrix();
	double transform[16] = { rot(0,0) , rot(1,0) , rot(2,0) , 0 ,
													 rot(0,1) , rot(1,1) , rot(2,1) , 0 ,
													 rot(0,2) , rot(1,2) , rot(2,2) , 0 ,
													 pos(0)   , pos(1)   , pos(2)   , 1 };
	glMultMatrixd(transform);
	glColor3f(color0, color1, color2);
	double cylinder[4][3] = { {-h-1.0, 0.0, 0.0} , {-h, 0.0, 0.0} , {0.0, 0.0, 0.0} ,
															 {1.0, 0.0, 0.0} };
	glePolyCylinder(4, cylinder, NULL, r);
	glPopMatrix();
}

void drawEndEffector(Vector3d pos, Matrix3d rot, double degrees, float color0, float color1, float color2) {
	glPushMatrix();
	double transform[16] = { rot(0,0) , rot(1,0) , rot(2,0) , 0 ,
													 rot(0,1) , rot(1,1) , rot(2,1) , 0 ,
													 rot(0,2) , rot(1,2) , rot(2,2) , 0 ,
													 pos(0)   , pos(1)   , pos(2)   , 1 };
	glMultMatrixd(transform);
			
	int pieces = 6;
	double h = 1.5;
	double start = -3;
	double handle_r = 1.2;
	double end = ((double) pieces)*h + start;
	double grab_offset = 12.0;
	
	glColor3f(0.3, 0.3, 0.0);
	double cylinder[4][3] = { {grab_offset-4.0, 0.0, 0.0} , {grab_offset-3.0, 0.0, 0.0} , {grab_offset, 0.0, 0.0} ,
														{grab_offset+1.0, 0.0, 0.0} };
	glePolyCylinder(4, cylinder, NULL, 1.6);
	
	glColor3f(color0, color1, color2);
	double grip_handle[4][3] = { {end-1.0, 0.0, 0.0} , {end, 0.0, 0.0} , {end+30.0, 0.0, 0.0} ,
															 {end+31.0, 0.0, 0.0} };
	glePolyCylinder(4, grip_handle, NULL, handle_r);

	glTranslatef(end, 0.0, 0.0);
	glRotatef(-degrees, 0.0, 0.0, 1.0);
	glTranslatef(-end, 0.0, 0.0);
	Matrix3d open_rot = (Matrix3d) Eigen::AngleAxisd(-degrees*M_PI/180.0, rot*Vector3d::UnitZ());
	Vector3d new_pos;
	Matrix3d new_rot;
	for (int piece=0; piece<pieces; piece++) {
		double r = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
		gleDouble grip_tip[4][3] = { {start+((double) piece)*h-1.0, r, 0.0} , {start+((double) piece)*h, r, 0.0} , {start+((double) piece+1)*h, r, 0.0} , {start+((double) piece+1)*h+1.0, r, 0.0} };
 		glePolyCylinder(4, grip_tip, NULL, r); 		
		new_rot = open_rot * rot;
		new_pos = open_rot * rot * Vector3d(-end, 0.0, 0.0) + rot * Vector3d(end, 0.0, 0.0) + pos;
	}
	
	glTranslatef(end, 0.0, 0.0);
	glRotatef(2*degrees, 0.0, 0.0, 1.0);
	glTranslatef(-end, 0.0, 0.0);
	for (int piece=0; piece<pieces; piece++) {
		double r = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
		gleDouble grip_tip[4][3] = { {start+((double) piece)*h-1.0, -r, 0.0} , {start+((double) piece)*h, -r, 0.0} , {start+((double) piece+1)*h, -r, 0.0} , {start+((double) piece+1)*h+1.0, -r, 0.0} };
 		glePolyCylinder(4, grip_tip, NULL, r);
 		new_rot = open_rot.transpose() * rot;
		new_pos = open_rot.transpose() * rot * Vector3d(-end, 0.0, 0.0) + rot * Vector3d(end, 0.0, 0.0) + pos;
	}
	glPopMatrix();
}

void drawGrip(Vector3d pos, Matrix3d rot, double radius, double degrees, float color0, float color1, float color2) {
	glPushMatrix();
	double transform[16] = { rot(0,0) , rot(1,0) , rot(2,0) , 0 ,
													 rot(0,1) , rot(1,1) , rot(2,1) , 0 ,
													 rot(0,2) , rot(1,2) , rot(2,2) , 0 ,
													 pos(0)   , pos(1)   , pos(2)   , 1 };
	glMultMatrixd(transform);
	glRotated(-90,0,0,1);
	glColor3f(color0, color1, color2);
	double grip_handle[4][3] = { {0.0, 9.0, 0.0} , {0.0,11.0, 0.0} , {0.0,19.0, 0.0} ,
															 {0.0,21.0, 0.0} };
	glePolyCylinder(4, grip_handle, NULL, 1);
	
	glTranslatef(0.0, 11.0, 0.0);
	glRotatef(-degrees, 1.0, 0.0, 0.0);
	glTranslatef(0.0, -11.0, 0.0);
	double grip_tip0[4][3] = { {0.0, 0.0,-2.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 2.0} ,
														 {0.0, 0.0, 4.0} };
	glePolyCylinder(4, grip_tip0, NULL, 0.95*radius);
	double grip_side0[8][3] = { {0.0, 0.0,-2.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 2.0} ,
															{0.0, 3.0, 5.0} , {0.0, 6.0, 5.0} , {0.0,11.0, 0.0} ,
															{0.0,13.0, 0.0} };
	glePolyCylinder(8, grip_side0, NULL, 1);
															
	glTranslatef(0.0, 11.0, 0.0);
	glRotatef(2*degrees, 1.0, 0.0, 0.0);
	glTranslatef(0.0, -11.0, 0.0);	
	double grip_tip1[4][3] = { {0.0, 0.0, 2.0} , {0.0, 0.0, 0.0} , {0.0, 0.0,-2.0} ,
														 {0.0, 0.0,-4.0} };
	glePolyCylinder(4, grip_tip1, NULL, 0.95*radius);
	double grip_side1[8][3] = { {0.0, 0.0, 2.0} , {0.0, 0.0, 0.0} , {0.0, 0.0,-2.0} ,
															{0.0, 3.0,-5.0} , {0.0, 6.0,-5.0} , {0.0,11.0, 0.0} ,
															{0.0,13.0, 0.0} };
	glePolyCylinder(8, grip_side1, NULL, 1);
	glPopMatrix();
}

void drawSphere(Vector3d position, float radius, float color0, float color1, float color2) {
	glPushMatrix();
	double transform[16] = {1,0,0,0,
													0,1,0,0,
													0,0,1,0,
													position(0), position(1), position(2), 1};
	glMultMatrixd(transform);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_COLOR_MATERIAL);
  glColor3f(color0, color1, color2);
  glutSolidSphere(radius, 20, 16);
  //glFlush ();
  glPopMatrix();
}

void drawCursor(double proxyxform[16], float color) {
  static const double kCursorRadius = 0.5;
  static const double kCursorHeight = 1.5;
  static const int kCursorTess = 4;
  
  GLUquadricObj *qobj = 0;

  glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
  glPushMatrix();

  if (!gCursorDisplayList) {
    gCursorDisplayList = glGenLists(1);
    glNewList(gCursorDisplayList, GL_COMPILE);
    qobj = gluNewQuadric();
            
    gluCylinder(qobj, 0.0, kCursorRadius, kCursorHeight,
                kCursorTess, kCursorTess);
    glTranslated(0.0, 0.0, kCursorHeight);
    gluCylinder(qobj, kCursorRadius, 0.0, kCursorHeight / 5.0,
                kCursorTess, kCursorTess);

    gluDeleteQuadric(qobj);
    glEndList();
  }
  
  // Get the proxy transform in world coordinates
 	glMultMatrixd(proxyxform);

  
  // Apply the local cursor scale factor.
  glScaled(gCursorScale, gCursorScale, gCursorScale);

  glEnable(GL_COLOR_MATERIAL);
  glColor3f(0.0, color, 1.0);

  glCallList(gCursorDisplayList);
  
  glPopMatrix();
  glPopAttrib();
}

void drawAxes(Vector3d pos, Matrix3d rot) {
	glPushMatrix();
	double transform[] = { rot(0,0) , rot(1,0) , rot(2,0) , 0 ,
												 rot(0,1) , rot(1,1) , rot(2,1) , 0 ,
												 rot(0,2) , rot(1,2) , rot(2,2) , 0 ,
												 pos(0)   , pos(1)   , pos(2)   , 1 };
	glMultMatrixd(transform);
	glBegin(GL_LINES);
	glEnable(GL_LINE_SMOOTH);
	glColor3d(1.0, 0.0, 0.0); //red
	glVertex3f(0.0, 0.0, 0.0); //x
	glVertex3f(10.0, 0.0, 0.0);
	glColor3d(0.0, 1.0, 0.0); //green
	glVertex3f(0.0, 0.0, 0.0); //y
	glVertex3f(0.0, 10.0, 0.0);
	glColor3d(0.0, 0.0, 1.0); //blue
	glVertex3f(0.0, 0.0, 0.0); //z
	glVertex3f(0.0, 0.0, 10.0);
	glEnd();
	glPopMatrix();
}

void labelAxes(Vector3d pos, Matrix3d rot) {
  glPushMatrix();
  Vector3d diff_pos = pos;
	double rotation_scale_factor = 20.0;
	Matrix3d rotations_project = rot*rotation_scale_factor;
	void * font = GLUT_BITMAP_HELVETICA_18;
	glColor3d(1.0, 0.0, 0.0); //red
	//glRasterPos3i(20.0, 0.0, -1.0);
	glRasterPos3i((float)(diff_pos(0)+rotations_project(0,0)), (float)(diff_pos(1)+rotations_project(1,0)), (float)(diff_pos(2)+rotations_project(2,0)));
	glutBitmapCharacter(font, 'X');
	glColor3d(0.0, 1.0, 0.0); //red
	//glRasterPos3i(0.0, 20.0, -1.0);
	glRasterPos3i((float)(diff_pos(0)+rotations_project(0,1)), (float)(diff_pos(1)+rotations_project(1,1)), (float)(diff_pos(2)+rotations_project(2,1)));
	glutBitmapCharacter(font, 'Y');
	glColor3d(0.0, 0.0, 1.0); //red
	//glRasterPos3i(-1.0, 0.0, 20.0);
	glRasterPos3i((float)(diff_pos(0)+rotations_project(0,2)), (float)(diff_pos(1)+rotations_project(1,2)), (float)(diff_pos(2)+rotations_project(2,2)));
	glutBitmapCharacter(font, 'Z');
  glPopMatrix();
}