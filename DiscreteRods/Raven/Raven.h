#ifndef Raven_h
#define Raven_h

#include <stdio.h>
#include <stdlib.h>

#include "forKRaven.h"
#include "invKRaven.h"
#include "utilFunc.h"

#include "DrawRobot.h"
#include "DrawObjRobot.h"
#include "obj.h"

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

#define NUM_MECH 4

class Raven
{
	public:
		Raven();
		~Raven();
	
		void draw();
	
	//protected:
		float posMatrix[NUM_MECH][HMATRIX_SIZE][HMATRIX_SIZE];
		float rtheta[NUM_MECH][7];
		int buttonState[4];
		ObjModel *left_base_model, *left_link1_model, *left_link2_model, *left_tool_model;
		ObjModel *right_base_model, *right_link1_model, *right_link2_model, *right_tool_model;
		ModelHolder leftHolder, rightHolder;
};

#endif //Raven_h
