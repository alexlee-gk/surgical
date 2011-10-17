#include "Raven.h"

Raven::Raven()
{
	// Solve forward and inverse kinematics
	float tip[NUM_OF_ROBOTS][HMATRIX_SIZE][HMATRIX_SIZE];

	//Create the angles and distances for the forward kinematics
	float t1[NUM_OF_ROBOTS];
	float t2[NUM_OF_ROBOTS];
	float t3[NUM_OF_ROBOTS];
	float d4[NUM_OF_ROBOTS];
	float t5[NUM_OF_ROBOTS];
	float t6[NUM_OF_ROBOTS];

	for (int i = 0; i < NUM_OF_ROBOTS; i++)
	{
		t1[i] = (float) (20.0*PI/180); 
		t2[i] = (float) (110.0*PI/180);
		t3[i] = (float) (0.0*PI/180);
		d4[i] = (float) (-100.0);
		t5[i] = (float) (0.0*PI/180);
		t6[i] = (float) (0.0*PI/180);
	}

	//Creates the forward kinematics
	forK(t1, t2, t3, d4, t5, t6, tip);

	//create the posMatrix for the inverse kinematics
	for (int i = 0; i < HMATRIX_SIZE; i++) {
		for (int j = 0; j < HMATRIX_SIZE; j++) {
			posMatrix[0][i][j] = tip[0][i][j];
			posMatrix[1][i][j] = tip[1][i][j];
			posMatrix[2][i][j] = tip[2][i][j];
			posMatrix[3][i][j] = tip[3][i][j];
		}
	}	

	//Print the results
	printf("The position matrix1 is:\n");
	printMatrix(posMatrix[0], "");
	printf("The position matrix2 is:\n");
	printMatrix(posMatrix[1], "");
	printf("The position matrix3 is:\n");
	printMatrix(posMatrix[2], "");
	printf("The position matrix4 is:\n");
	printMatrix(posMatrix[3], "");

	//Calculate the inverse Kinematics
	invK(posMatrix[0], rtheta[0], 1);
	invK(posMatrix[1], rtheta[1], 2);
	invK(posMatrix[2], rtheta[2], 3);
	invK(posMatrix[3], rtheta[3], 4);

	for (int r = 0; r < NUM_MECH; r++)
	{
		printf("iks[%d]: %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f\n\n", r, rtheta[r][0]*180/M_PI, rtheta[r][1]*180/M_PI, rtheta[r][2]*180/M_PI, rtheta[r][3], rtheta[r][4]*180/M_PI, rtheta[r][5]*180/M_PI, rtheta[r][6]*180/M_PI);
	}
	
	
	char* memory1 = NULL;
	size_t bytes = ObjLoadFile("Raven/objFiles/base_left.obj", &memory1);
	left_base_model = ObjLoadModel(memory1, bytes);
	printf("Left base has: %d faces!\n", left_base_model->nTriangle);

	char* memory2 = NULL;
	size_t bytes2 = ObjLoadFile("Raven/objFiles/base_right.obj", &memory2);
	right_base_model = ObjLoadModel(memory2, bytes2);
	printf("Right base has: %d faces!\n", left_base_model->nTriangle);

	char* memory3 = NULL;
	size_t bytes3 = ObjLoadFile("Raven/objFiles/link1_left.obj", &memory3);
	left_link1_model = ObjLoadModel(memory3, bytes3);
	printf("Left link1 has: %d faces!\n", left_link1_model->nTriangle);

	char* memory4 = NULL;
	size_t bytes4 = ObjLoadFile("Raven/objFiles/link1_right.obj", &memory4);
	right_link1_model = ObjLoadModel(memory4, bytes4);
	printf("Right link1 has: %d faces!\n", right_link1_model->nTriangle);

	char* memory5 = NULL;
  size_t bytes5 = ObjLoadFile("Raven/objFiles/link2_left.obj", &memory5);
  left_link2_model = ObjLoadModel(memory5, bytes5);
  printf("Left link2 has: %d faces!\n", left_link2_model->nTriangle);

	char* memory6 = NULL;
  size_t bytes6 = ObjLoadFile("Raven/objFiles/link2_right.obj", &memory6);
  right_link2_model = ObjLoadModel(memory6, bytes6);
  printf("Right link2 has: %d faces!\n", right_link2_model->nTriangle);

	char* memory7 = NULL;
	size_t bytes7 = ObjLoadFile("Raven/objFiles/tool_left.obj", &memory7);
	left_tool_model = ObjLoadModel(memory7, bytes7);
	printf("Left tool has: %d faces!\n", left_tool_model->nTriangle);

	char* memory8 = NULL;
	size_t bytes8 = ObjLoadFile("Raven/objFiles/tool_right.obj", &memory8);
	right_tool_model = ObjLoadModel(memory8, bytes8);
	printf("Right tool has: %d faces!\n", right_tool_model->nTriangle);

	leftHolder.base = left_base_model;
	leftHolder.lower_link = left_link1_model;
	leftHolder.upper_link = left_link2_model;
	leftHolder.tool = left_tool_model;
	rightHolder.base = right_base_model;
	rightHolder.lower_link = right_link1_model;
	rightHolder.upper_link = right_link2_model;
	rightHolder.tool = right_tool_model;
}

Raven::~Raven()
{
	free(left_base_model->NormalArray);
	free(left_base_model->TexCoordArray);
	free(left_base_model->TriangleArray);
	free(left_base_model->VertexArray);
	free(left_base_model);
	free(right_base_model->NormalArray);
	free(right_base_model->TexCoordArray);
	free(right_base_model->TriangleArray);
	free(right_base_model->VertexArray);
	free(right_base_model);
	free(left_link1_model->NormalArray);
	free(left_link1_model->TexCoordArray);
	free(left_link1_model->TriangleArray);
	free(left_link1_model->VertexArray);
	free(left_link1_model);
	free(right_link1_model->NormalArray);
	free(right_link1_model->TexCoordArray);
	free(right_link1_model->TriangleArray);
	free(right_link1_model->VertexArray);
	free(right_link1_model);
	free(left_link2_model->NormalArray);
	free(left_link2_model->TexCoordArray);
	free(left_link2_model->TriangleArray);
	free(left_link2_model->VertexArray);
	free(left_link2_model);
	free(right_link2_model->NormalArray);
	free(right_link2_model->TexCoordArray);
	free(right_link2_model->TriangleArray);
	free(right_link2_model->VertexArray);
	free(right_link2_model);
	free(left_tool_model->NormalArray);
	free(left_tool_model->TexCoordArray);
	free(left_tool_model->TriangleArray);
	free(left_tool_model->VertexArray);
	free(left_tool_model);
	free(right_tool_model->NormalArray);
	free(right_tool_model->TexCoordArray);
	free(right_tool_model->TriangleArray);
	free(right_tool_model->VertexArray);
	free(right_tool_model);
}

void Raven::draw()
{
	static GLfloat mat_ambient[]    = { 0.7f, 0.7f, 0.7f, 1.0 };
	static GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 1.0 };
	static GLfloat mat_specular[]   = { 1.0f, 1.0f, 1.0f, 1.0 };
	static GLfloat high_shininess[] = { 100.0 };

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);
  
  glEnable(GL_COLOR_MATERIAL);

//	printMatrix(posMatrix[0], "posMatrix[0]");	
//	printMatrix(posMatrix[1], "posMatrix[1]");	
//	printMatrix(posMatrix[2], "posMatrix[2]");
//	printMatrix(posMatrix[3], "posMatrix[3]");

	//Calculate the inverse Kinematics
	invK(posMatrix[0], rtheta[0], 1);
	invK(posMatrix[1], rtheta[1], 2);
	invK(posMatrix[2], rtheta[2], 3);
	invK(posMatrix[3], rtheta[3], 4);

	glPushMatrix();
	glTranslatef(0, 0, -35);
	glRotatef(180,0,1,0);
	
	for (int r = 0; r < NUM_MECH; r++)
	{
		glPushMatrix();
		//printf("%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f\n", rtheta[0], rtheta[1], rtheta[2], rtheta[3], rtheta[4], rtheta[5], rtheta[6]);
		//printf("da: %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f\n\n", rtheta[r][0]*180/M_PI, rtheta[r][1]*180/M_PI, rtheta[r][2]*180/M_PI, rtheta[r][3], rtheta[r][4]*180/M_PI, rtheta[r][5]*180/M_PI, rtheta[r][6]*180/M_PI);
		if (r == 0)
			DrawObjRobot(rtheta[r], r, 50.0f, 0.0f, -180.0f, buttonState[r], rightHolder);
		else if (r == 1)
			DrawObjRobot(rtheta[r], r, -50.0f, 0.0f, -180.0f, buttonState[r], leftHolder);
		else if (r == 2)
			DrawObjRobot(rtheta[r], r, 50.0f, 0.0f, 180.0f, rightHolder);
		else if (r == 3)
			DrawObjRobot(rtheta[r], r, -50.0f, 0.0f, 180.0f, leftHolder);
		glPopMatrix();
	}
	glPopMatrix();
}

