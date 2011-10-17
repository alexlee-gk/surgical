
#include "invKRaven.h"

#define ABS(x) ( (x) > (0) ? (x) : (-(x)) )

//Implement the inverse kinematics
int invK(float tip[][HMATRIX_SIZE], float *resultVector, int robotNum)
{
	int has_solution = 0;
	float tip_inv[HMATRIX_SIZE][HMATRIX_SIZE];
	float a5 = VALUEA5;
	float a1 = DESIGN_ANGLE1;
	float b1 = DESIGN_ANGLE2;
	//printf("the tip matrix\n");
	//printMatrix(tip);	

	invOrthMatrix(tip,tip_inv);
	//printf("the inverse recieved matrix\n");
	//printMatrix(tip_inv);
	float Px_inv = tip_inv[0][3];
	float Py_inv = tip_inv[1][3];
	float Pz_inv = tip_inv[2][3];
	float Pxyz = SQ(tip[0][3]) + SQ(tip[1][3]) + SQ(tip[2][3]);
	
	//There are four solutions for d4
	float d41=sqrt((SQ(a5)+Pxyz+2*sqrt((SQ(a5)*Pxyz-SQ(Py_inv)*SQ(a5)))));
	float d42=-sqrt((SQ(a5)+Pxyz+2*sqrt((SQ(a5)*Pxyz-SQ(Py_inv)*SQ(a5)))));
	float d43=sqrt((SQ(a5)+Pxyz-2*sqrt((SQ(a5)*Pxyz-SQ(Py_inv)*SQ(a5)))));
	float d44=-1*sqrt((SQ(a5)+Pxyz-2*sqrt((SQ(a5)*Pxyz-SQ(Py_inv)*SQ(a5)))));

	//We choose solution number 4
	float dd4 = d44;
	//printf("the dd4 = %f\n",dd4);
	float s5 = (Py_inv/dd4);
	if ((1 + EPS2 > ABS(s5)) &&(ABS(s5) > 1 - EPS2)) 
	{
		s5 = ABS(s5)/s5;
	}
	float c5 = 0;
	float th5 = asin(s5);
	//Solved for theta5
	float th5_deg = th5*180/PI;
	//printf("th5_deg = %f\n",th5_deg);
	

	float c6 = 0;
	float s6 =0;

	c5=sqrt(1-SQ(s5));
	if ((robotNum == 1)||(robotNum == 3))
	{
		c6 = Pz_inv/(-1*c5*dd4+a5);
		s6 = Px_inv/(-1*c5*dd4+a5);
	} 
	if ((robotNum == 2)||(robotNum == 4))
	{
		c6 = Pz_inv/(-1*c5*dd4+a5);
		s6 = -Px_inv/(-1*c5*dd4+a5);
	}  
	
	float th6 = atan2(s6,c6);
	//Solved for th6
	float th6_deg = th6*180/PI;
	//printf("th6_deg = %f\n",th6_deg);
	float T46[HMATRIX_SIZE][HMATRIX_SIZE];
	float T46_inv[HMATRIX_SIZE][HMATRIX_SIZE];
	float T13[HMATRIX_SIZE][HMATRIX_SIZE];
	float T23[HMATRIX_SIZE][HMATRIX_SIZE];
	float temp[HMATRIX_SIZE][HMATRIX_SIZE];
	float temp_inv[HMATRIX_SIZE][HMATRIX_SIZE];
	float T1[HMATRIX_SIZE][HMATRIX_SIZE];

		 
	if ((robotNum == 1)||(robotNum == 3))
	{
		T46[0][0] = -s5*s6; T46[0][1] = -c5; T46[0][2] = -s5*c6; T46[0][3] = a5*s5;
		T46[1][0] = c6; T46[1][1] = 0; T46[1][2] = -s6; T46[1][3] = 0;
		T46[2][0] = c5*s6; T46[2][1] = -s5; T46[2][2] = c5*c6; T46[2][3] = -a5*c5+dd4;
		T46[3][0] = 0; T46[3][1] = 0; T46[3][2] = 0; T46[3][3] = 1;
	}

	if ((robotNum == 2)||(robotNum == 4))
	{
		T46[0][0] = -s5*s6; T46[0][1] = c5; T46[0][2] = s5*c6; T46[0][3] = -a5*s5;
		T46[1][0] = -c6; T46[1][1] = 0; T46[1][2] = -s6; T46[1][3] = 0;
		T46[2][0] = -c5*s6; T46[2][1] = -s5; T46[2][2] = c5*c6; T46[2][3] = -a5*c5+dd4;
		T46[3][0] = 0; T46[3][1] = 0; T46[3][2] = 0; T46[3][3] = 1;
	}
	
	invOrthMatrix(T46,T46_inv);
	mulMatrix(tip,T46_inv,T13);
	//printf("T13 matrix\n");
	//printMatrix(T13);
	float c2 = (cos(a1)*cos(b1)+ T13[2][2])/(sin(a1)*sin(b1));
	float s2 = 0;
	//printf("ABS of c2 is %f\n",c2);
	if (ABS(c2) > 1 + EPS1) 
	{
		printf("Error: ABS(c2) > 1.0 + EPS1 \n");
		return has_solution;
	}
	s2 = sqrt(1-SQ(c2));
	float th2 = atan2(s2,c2);
	//Solved for theta2
	float th2_deg = th2*180/PI;
	//printf("th2_deg = %f\n",th2_deg);

	float a = sin(a1)*s2;
	float b = sin(a1)*c2*cos(b1)+cos(a1)*sin(b1);
	float c = T13[2][1];
	float dd = SQ(b)+SQ(a)-SQ(c);
	//printf("dd = %f\n",dd);
	if(dd<0) 
	{
		if (ABS(dd)< EPS1*2)
		{
			dd=0;
		} else {
			printf("Error: ABS(b*b+a*a-c*c)<EPS1, %f < %f", ABS(dd), EPS1);
			return has_solution;
		}
	}

	if(ABS(a+c) < EPS1)
	{
		printf("Error: ABS(a+c) < %f", EPS1);
	}

	//There are two solutions for th3
	float th31 = 2*atan((b+sqrt(dd))/(a+c));
	float th31_deg = th31*180/PI;
	float th32 = 2*atan((b-sqrt(dd))/(a+c));
	float th32_deg = th31*180/PI;

	//We can choose by selection of other unit in matrix
	float s31 = sin(th31);
	float c31 = cos(th31);
	float s32 = sin(th32);
	float c32 = cos(th32);
	float T13_calc1 = 0;
	float T13_calc2 = 0;

	if ((robotNum == 1)||(robotNum == 3))
	{
		T13_calc1 = -sin(a1)*s2*s31+(sin(a1)*c2*cos(b1)+cos(a1)*sin(b1))*c31;
		T13_calc2 = -sin(a1)*s2*s32+(sin(a1)*c2*cos(b1)+cos(a1)*sin(b1))*c32;
	}

	if ((robotNum == 2)||(robotNum == 4))
	{
		T13_calc1 = sin(a1)*s2*s31-(sin(a1)*c2*cos(b1)+cos(a1)*sin(b1))*c31;
		T13_calc2 = sin(a1)*s2*s32-(sin(a1)*c2*cos(b1)+cos(a1)*sin(b1))*c32;
	}
	
	float th3 = 0;
	/*if ((T13_calc1 < T13(3, 1) + EPS1) && (T13_calc1 > T13(3, 1) - EPS1))
		th3 = th31;
	elseif ((T13_calc2 < T13(3, 1) + EPS1) && (T13_calc2 > T13(3, 1) - EPS1))
		th3 = th32;
	else*/

/*
	if ( ABS(T13_calc1 - T13[2][0]) < EPS1 )
	{
		th3 = th31;
	}
	else if ( ABS(T13_calc2 - T13[2][0]) < EPS1 )
	{
		th3 = th32;
	}
	else
	{
		RTDEBUG(MSG_ERROR, "ik: t3 : T13c1: %f, T13c2: %f | T13[2,0]: %f> %f %f", T13_calc1, T13_calc2, T13[2][0], th31 RAD2DEG, th32 RAD2DEG);
		printMatrix4x4(MSG_ERROR, tip, "tip");
		//RTDEBUG(MSG_ERROR, "ik: dd4: %f, th5: %f, th6: %f, th2: %f", dd4, th5, th6, th2);
		return has_solution;
	}
*/

	if ( ABS(T13_calc1 - T13[2][0]) < ABS(T13_calc2 - T13[2][0]) )
	{
		th3 = th31;
	}
	else
	{
		th3 = th32;
	}

	//th3 final, Solved for th3
	float th3_deg = th3*180/PI;
	//printf("th3_deg = %f\n",th3_deg);

	float c3 = cos(th3);
	float s3 = sin(th3);

	if ((robotNum == 1)||(robotNum == 3))
	{
		T23[0][0] = c2*s3+s2*cos(b1)*c3;
		T23[0][1] = -c2*c3+s2*cos(b1)*s3;
		T23[0][2] = s2*sin(b1);
		T23[0][3] = 0;
		
		T23[1][0] = -s2*s3+c2*cos(b1)*c3;
		T23[1][1] = s2*c3+c2*cos(b1)*s3;
		T23[1][2] = c2*sin(b1);
		T23[1][3] = 0;
		
		T23[2][0] = -sin(b1)*c3;
		T23[2][1] = -sin(b1)*s3;
		T23[2][2] = cos(b1);
		T23[2][3] = 0;

		T23[3][0] = 0;
		T23[3][1] = 0;
		T23[3][2] = 0;
		T23[3][3] = 1;
	}

	if ((robotNum == 2)||(robotNum == 4))
	{
		T23[0][0] = c2*s3+s2*cos(b1)*c3;
		T23[0][1] = c2*c3-s2*cos(b1)*s3;
		T23[0][2] = -s2*sin(b1);
		T23[0][3] = 0;
		
		T23[1][0] = s2*s3-c2*cos(b1)*c3;
		T23[1][1] = s2*c3+c2*cos(b1)*s3;
		T23[1][2] = c2*sin(b1);
		T23[1][3] = 0;
		
		T23[2][0] = sin(b1)*c3;
		T23[2][1] = -sin(b1)*s3;
		T23[2][2] = cos(b1);
		T23[2][3] = 0;

		T23[3][0] = 0;
		T23[3][1] = 0;
		T23[3][2] = 0;
		T23[3][3] = 1;
	}
	
	mulMatrix(T23,T46,temp);
	invOrthMatrix(temp, temp_inv);
	mulMatrix(tip,temp_inv,T1);
	//printMatrix(T1);

	float c1 = 0;
	float s1 = 0;

	if ((robotNum == 1)||(robotNum == 3))
	{
		c1 = T1[0][0];
		s1 = T1[1][0];
	}

	if ((robotNum == 2)||(robotNum == 4))
	{
		c1 = -T1[0][0];
		s1 = T1[1][0];
	}

	//th1 final
	float th1 = atan2(s1,c1);
	//solved for theta1
	float th1_deg = th1*180/PI;

	//enter the result to the vector
	resultVector[0] = th1;
	resultVector[1] = th2;
	resultVector[2] = th3;
	resultVector[3] = dd4;
	resultVector[4] = th5;
	resultVector[5] = th6;
	resultVector[6] = (float) (10.0 * M_PI/180);

	return has_solution;
}




