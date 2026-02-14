#include <math.h>
#include "serial.h"
#include "moore_penrose_pseudoinverse.c"

#define PI		3.14159265358
#define DTR 	PI/180.0				   // Conversi degree to radian
#define RTD 	180.0/PI				   // Conversi degree to radian

// #define L1	0.3   // link1
// #define L2	0.2   // link2

struct global_odom{
	float x;
	float y;
	float z;
}end_effector;

struct global_rotation{
	float q1;
	float q2;
	float q3;
	float q4;
	float q5;
	float q6;
} jacob_inverse;

struct global_arm{
	float tetha1;
	float tetha2;
	float tetha3;
	float tetha4;
	float tetha5;
	float tetha6;
	bool gripper_state;
	float d0;
	float a1;
	float a2;
	float a3;
	float d4;
	float a5;
	float d6;
} arm_data_now;

// buat moore_penros
const realtype rcond = 1E-15;

float q1;
float q2;
float q3;
float q4;
float q5;
float q6;

float objx=0.3;
float objy=0.5;

float D1 = 0.1;
float A1 = 0.02;
float A2 = 0.13;
float A3 = 0.02;
float D4 = 0.15;
float A5 = 0.00;
float D6 = 0.125;

float Kp = 0.5;
float Kd = 0.01;

float t = 0.;
float dt = 0.02;

// int k = 0;
// int timewanted = 5;
// int freq = 50;

// JOINT SPACE CONTROL VARIABLE
bool jalan_joint_space = false;

float q1awal, q2awal, q3awal, q4awal, q5awal, q6awal;
float q1final, q2final, q3final, q4final, q5final, q6final;
float traj_q1, traj_q2, traj_q3, traj_q4, traj_q5, traj_q6;

float error_q1 = 0, error_q2 = 0, error_q3 = 0;
float error_q4 = 0, error_q5 = 0, error_q6 = 0;

float error_bef_q1 = 0, error_bef_q2 = 0, error_bef_q3 = 0;
float error_bef_q4 = 0, error_bef_q5 = 0, error_bef_q6 = 0;

float diff_error_q1 = 0, diff_error_q2 = 0, diff_error_q3 = 0;
float diff_error_q4 = 0, diff_error_q5 = 0, diff_error_q6 = 0;

// CONTROL EFFORT VARIABLE RELATED
float q1_dot, q2_dot, q3_dot, q4_dot, q5_dot, q6_dot;
float q1_ddot, q2_ddot, q3_ddot, q4_ddot, q5_ddot, q6_ddot;

// global variables
float xd, yd, zd;        // target position
float x_prev, y_prev, z_prev; // previous position

struct global_odom target_pos;
struct global_odom prev_pos;

typedef struct{
	float q1;
	float q2;
} Tdata;

int set_Arm(float sudut1, float sudut2, float sudut3, float sudut4, float sudut5, float sudut6, bool gripper7){
	printf("sudut1 = %.3f", (float)sudut1);
	// Set First Link
	if (sudut1 < -45 || sudut1 > 45)
		return 0; // limit check
	else{
		arm_data_now.tetha1 = sudut1;
		q1 = (float)sudut1 * DTR;
		printf(" q1 = %.3f", (float)q1 * RTD);
	}
	printf(" Success\n");

	// Set Second Link
	printf("sudut2 = %.3f", (float)sudut2);
	if (sudut2 < 45 || sudut2 > 135)
		return 0; // limit check
	else{
		arm_data_now.tetha2 = sudut2;
		q2 = (float)sudut2 * DTR;
		printf(" q2 = %.3f", (float)q2 * RTD);
	}
	printf(" Success\n");
	
	// Set Third Link
	printf("sudut3 = %.3f", (float)sudut3);
	if (sudut3 < -45 || sudut3 > 45)
		return 0; // limit check
	else{
		arm_data_now.tetha3 = sudut3;
		q3 = (float)sudut3 * DTR;
		printf(" q3 = %.3f", (float)q3 * RTD);
	}
	printf(" Success\n");

	// Set Fourth Link
	printf("sudut4 = %.3f", (float)sudut4);
	if (sudut4 < -45 || sudut4 > 135)
		return 0; // limit check
	else{
		arm_data_now.tetha4 = sudut4;
		q4 = (float)sudut4 * DTR;
		printf(" q4 = %.3f", (float)q4 * RTD);
	}
	printf(" Success\n");

	// Set Fifth Link
	printf("sudut5 = %.3f", (float)sudut5);
	if (sudut5 < -90 || sudut5 > 90)
		return 0; // limit check
	else{
		arm_data_now.tetha5 = sudut5;
		q5 = (float)sudut5 * DTR;
		printf(" q5 = %.3f", (float)q5 * RTD);
	}
	printf(" Success\n");

	// Set Sixth Link
	printf("sudut6 = %.3f", (float)sudut6);
	if (sudut6 < -45 || sudut6 > 45)
		return 0; // limit check
	else{
		arm_data_now.tetha6 = sudut6;
		q6 = (float)sudut6 * DTR;
		printf(" q6 = %.3f", (float)q6 * RTD);
	}
	printf(" Success\n");

	arm_data_now.gripper_state = gripper7;

	return 1;
}

// Alternative: Direct set_Arm with PD control
int set_Arm_PD(float target_q1, float target_q2, float target_q3, 
               float target_q4, float target_q5, float target_q6, bool gripper) {
    
    // Calculate errors
    error_q1 = target_q1 - arm_data_now.tetha1;
    error_q2 = target_q2 - arm_data_now.tetha2;
    error_q3 = target_q3 - arm_data_now.tetha3;
    error_q4 = target_q4 - arm_data_now.tetha4;
    error_q5 = target_q5 - arm_data_now.tetha5;
    error_q6 = target_q6 - arm_data_now.tetha6;
    
    // Calculate derivative of errors
    diff_error_q1 = (error_q1 - error_bef_q1) / dt;
    diff_error_q2 = (error_q2 - error_bef_q2) / dt;
    diff_error_q3 = (error_q3 - error_bef_q3) / dt;
    diff_error_q4 = (error_q4 - error_bef_q4) / dt;
    diff_error_q5 = (error_q5 - error_bef_q5) / dt;
    diff_error_q6 = (error_q6 - error_bef_q6) / dt;
    
    // PD control
    float new_q1 = arm_data_now.tetha1 + Kp * error_q1 + Kd * diff_error_q1;
    float new_q2 = arm_data_now.tetha2 + Kp * error_q2 + Kd * diff_error_q2;
    float new_q3 = arm_data_now.tetha3 + Kp * error_q3 + Kd * diff_error_q3;
    float new_q4 = arm_data_now.tetha4 + Kp * error_q4 + Kd * diff_error_q4;
    float new_q5 = arm_data_now.tetha5 + Kp * error_q5 + Kd * diff_error_q5;
    float new_q6 = arm_data_now.tetha6 + Kp * error_q6 + Kd * diff_error_q6;
    
    // Update previous errors
    error_bef_q1 = error_q1;
    error_bef_q2 = error_q2;
    error_bef_q3 = error_q3;
    error_bef_q4 = error_q4;
    error_bef_q5 = error_q5;
    error_bef_q6 = error_q6;
    
    // Call original set_Arm with controlled angles
    return set_Arm(new_q1, new_q2, new_q3, new_q4, new_q5, new_q6, gripper);
}

// Original set_arm_inv_Jacob (without PD control)
int set_arm_inv_Jacob(bool gripper){
    float target_q1 = arm_data_now.tetha1 + jacob_inverse.q1;
    float target_q2 = arm_data_now.tetha2 + jacob_inverse.q2;
    float target_q3 = arm_data_now.tetha3 + jacob_inverse.q3;
    float target_q4 = arm_data_now.tetha4 + jacob_inverse.q4;
    float target_q5 = arm_data_now.tetha5 + jacob_inverse.q5;
    float target_q6 = arm_data_now.tetha6 + jacob_inverse.q6;
    
    printf("q dot calculation success\n");
    return set_Arm(target_q1, target_q2, target_q3, target_q4, target_q5, target_q6, gripper);
}

struct global_odom forwardKinematics(){
	struct global_odom forward_kine_result;

	// temporary variable to reduce calculation
	float t1 = (float)arm_data_now.tetha1 * DTR;
	float t2 = (float)arm_data_now.tetha2 * DTR;
	float t3 = (float)arm_data_now.tetha3 * DTR;
	float t4 = (float)arm_data_now.tetha4 * DTR;
	float t5 = (float)arm_data_now.tetha5 * DTR;
	float t6 = (float)arm_data_now.tetha6 * DTR;

	float c1 = cos(t1), s1 = sin(t1);
	float c2 = cos(t2), s2 = sin(t2);
	float c3 = cos(t3), s3 = sin(t3);
	float c4 = cos(t4), s4 = sin(t4);
	float c5 = cos(t5), s5 = sin(t5);

	float c23 = cos(t2 + t3);
	float s23 = sin(t2 + t3);

	float c45 = cos(t4 + t5);
	float s45 = sin(t4 + t5);
	float c4m5 = cos(t4 - t5);
	float s4m5 = sin(t4 - t5);

	forward_kine_result.x =
		arm_data_now.a1 * c1
		+ arm_data_now.a3 * c23 * c1
		+ arm_data_now.d4 * s23 * c1
		+ arm_data_now.a2 * c1 * c2
		+ arm_data_now.d6 * s23 * c1 * c5
		- arm_data_now.a5 * s23 * c1 * s5
		+ arm_data_now.a5 * c5 * s1 * s4
		+ arm_data_now.d6 * s1 * s4 * s5
		+ arm_data_now.d6 * c1 * c2 * c3 * c4 * s5
		- arm_data_now.a5 * c1 * c4 * c5 * s2 * s3
		- arm_data_now.d6 * c1 * c4 * s2 * s3 * s5
		+ arm_data_now.a5 * c1 * c2 * c3 * c4 * c5;

	forward_kine_result.y =
		arm_data_now.a1 * s1
		+ arm_data_now.a3 * c23 * s1
		+ arm_data_now.d4 * s23 * s1
		+ arm_data_now.a2 * c2 * s1
		+ arm_data_now.d6 * s23 * c5 * s1
		- arm_data_now.a5 * s23 * s1 * s5
		- arm_data_now.a5 * c1 * c5 * s4
		- arm_data_now.d6 * c1 * s4 * s5
		+ arm_data_now.a5 * c2 * c3 * c4 * c5 * s1
		+ arm_data_now.d6 * c2 * c3 * c4 * s1 * s5
		- arm_data_now.a5 * c4 * c5 * s1 * s2 * s3
		- arm_data_now.d6 * c4 * s1 * s2 * s3 * s5;
	
	forward_kine_result.z =
		arm_data_now.d0
		- arm_data_now.d4 * c23
		+ arm_data_now.a3 * s23
		+ arm_data_now.a2 * s2
		+ 0.5f * arm_data_now.a5 * c45 * s23
		+ 0.5f * arm_data_now.d6 * s23 * s45
		- arm_data_now.d6 * c23 * c5
		+ arm_data_now.a5 * c23 * s5
		+ 0.5f * arm_data_now.a5 * c4m5 * s23
		- 0.5f * arm_data_now.d6 * s4m5 * s23;

	return forward_kine_result;
}

struct global_rotation inverseJacobian(float x_dot, float y_dot, float z_dot){
	struct global_rotation inv_jacob_result;

	printf("arm_data_now.tetha1 = %f\n", arm_data_now.tetha1);
	printf("arm_data_now.tetha2 = %f\n", arm_data_now.tetha2);
	printf("arm_data_now.tetha3 = %f\n", arm_data_now.tetha3);
	printf("arm_data_now.tetha4 = %f\n", arm_data_now.tetha4);
	printf("arm_data_now.tetha5 = %f\n", arm_data_now.tetha5);
	printf("arm_data_now.tetha6 = %f\n", arm_data_now.tetha6);

	// temporary variable to reduce calculation
	float temp_tetha1 = (float)arm_data_now.tetha1 * DTR;
	float temp_tetha2 = (float)arm_data_now.tetha2 * DTR;
	float temp_tetha3 = (float)arm_data_now.tetha3 * DTR;
	float temp_tetha4 = (float)arm_data_now.tetha4 * DTR;
	float temp_tetha5 = (float)arm_data_now.tetha5 * DTR;
	float temp_tetha6 = (float)arm_data_now.tetha6 * DTR;

	// Trigonometric shortcuts (VERY IMPORTANT for speed)
	float s1 = sinf(temp_tetha1);
	float c1 = cosf(temp_tetha1);
	float s2 = sinf(temp_tetha2);
	float c2 = cosf(temp_tetha2);
	float s3 = sinf(temp_tetha3);
	float c3 = cosf(temp_tetha3);
	float s4 = sinf(temp_tetha4);
	float c4 = cosf(temp_tetha4);
	float s5 = sinf(temp_tetha5);
	float c5 = cosf(temp_tetha5);

	float s23 = sinf(temp_tetha2 + temp_tetha3);
	float c23 = cosf(temp_tetha2 + temp_tetha3);

	// Jacobian elements J11 ... J36
	float J11 =
		-arm_data_now.a1 * s1
		- arm_data_now.a2 * s1 * c2
		- arm_data_now.a3 * s1 * c23
		+ arm_data_now.a5 * s1 * s5 * s23
		- arm_data_now.a5 * s1 * c4 * c5 * c23
		+ arm_data_now.a5 * s4 * c1 * c5
		- arm_data_now.d4 * s1 * s23
		- arm_data_now.d6 * s1 * s5 * c4 * c23
		- arm_data_now.d6 * s1 * s23 * c5
		+ arm_data_now.d6 * s4 * s5 * c1;

	float J21 =
		arm_data_now.a1 * c1
		+ arm_data_now.a2 * c1 * c2
		+ arm_data_now.a3 * c1 * c23
		+ arm_data_now.a5 * s1 * s4 * c5
		- arm_data_now.a5 * s5 * s23 * c1
		+ arm_data_now.a5 * c1 * c4 * c5 * c23
		+ arm_data_now.d4 * s23 * c1
		+ arm_data_now.d6 * s1 * s4 * s5
		+ arm_data_now.d6 * s5 * c1 * c4 * c23
		+ arm_data_now.d6 * s23 * c1 * c5;

	float J31 = 0;

	float J12 =
		(-arm_data_now.a2 * s2
		- arm_data_now.a3 * s23
		- arm_data_now.a5 * s5 * c23
		- arm_data_now.a5 * s23 * c4 * c5
		+ arm_data_now.d4 * c23
		- arm_data_now.d6 * s5 * s23 * c4
		+ arm_data_now.d6 * c5 * c23) * c1;

	float J22 =
		(-arm_data_now.a2 * s2
		- arm_data_now.a3 * s23
		- arm_data_now.a5 * s5 * c23
		- arm_data_now.a5 * s23 * c4 * c5
		+ arm_data_now.d4 * c23
		- arm_data_now.d6 * s5 * s23 * c4
		+ arm_data_now.d6 * c5 * c23) * s1;

	float J32 =
		arm_data_now.a2 * c2
		+ arm_data_now.a3 * c23
		- arm_data_now.a5 * s5 * s23
		+ arm_data_now.a5 * c4 * c5 * c23
		+ arm_data_now.d4 * s23
		+ arm_data_now.d6 * s5 * c4 * c23
		+ arm_data_now.d6 * s23 * c5;

	float J13 =
		(-arm_data_now.a3 * s23
		- arm_data_now.a5 * s5 * c23
		- arm_data_now.a5 * s23 * c4 * c5
		+ arm_data_now.d4 * c23
		- arm_data_now.d6 * s5 * s23 * c4
		+ arm_data_now.d6 * c5 * c23) * c1;

	float J23 =
		(-arm_data_now.a3 * s23
		- arm_data_now.a5 * s5 * c23
		- arm_data_now.a5 * s23 * c4 * c5
		+ arm_data_now.d4 * c23
		- arm_data_now.d6 * s5 * s23 * c4
		+ arm_data_now.d6 * c5 * c23) * s1;

	float J33 =
		arm_data_now.a3 * c23
		- arm_data_now.a5 * s5 * s23
		+ arm_data_now.a5 * c4 * c5 * c23
		+ arm_data_now.d4 * s23
		+ arm_data_now.d6 * s5 * c4 * c23
		+ arm_data_now.d6 * s23 * c5;

	float J14 =
		arm_data_now.a5 * s1 * c4 * c5
		- arm_data_now.a5 * s4 * c1 * c5 * c23
		+ arm_data_now.d6 * s1 * s5 * c4
		- arm_data_now.d6 * s4 * s5 * c1 * c23;

	float J24 =
		-arm_data_now.a5 * s1 * s4 * c5 * c23
		- arm_data_now.a5 * c1 * c4 * c5
		- arm_data_now.d6 * s1 * s4 * s5 * c23
		- arm_data_now.d6 * s5 * c1 * c4;
	
	
	float J34 =
		-(arm_data_now.a5 * c5 + arm_data_now.d6 * s5) * s4 * s23;

	float J15 =
		-arm_data_now.a5 * s1 * s4 * s5
		- arm_data_now.a5 * s5 * c1 * c4 * c23
		- arm_data_now.a5 * s23 * c1 * c5
		+ arm_data_now.d6 * s1 * s4 * c5
		- arm_data_now.d6 * s5 * s23 * c1
		+ arm_data_now.d6 * c1 * c4 * c5 * c23;

	float J25 =
		-arm_data_now.a5 * s1 * s5 * c4 * c23
		- arm_data_now.a5 * s1 * s23 * c5
		+ arm_data_now.a5 * s4 * s5 * c1
		- arm_data_now.d6 * s1 * s5 * s23
		+ arm_data_now.d6 * s1 * c4 * c5 * c23
		- arm_data_now.d6 * s4 * c1 * c5;

	float J35 =
		-arm_data_now.a5 * s5 * s23 * c4
		+ arm_data_now.a5 * c5 * c23
		+ arm_data_now.d6 * s5 * c23
		+ arm_data_now.d6 * s23 * c4 * c5;

	float J16 = 0;
	float J26 = 0;
	float J36 = 0;

	// deklarasi variabel dan alokasi memori
	gsl_matrix *Jacobian = gsl_matrix_alloc(3,6);

	// baris pertama
	gsl_matrix_set(Jacobian, 0, 0, J11);
	gsl_matrix_set(Jacobian, 0, 1, J12);
	gsl_matrix_set(Jacobian, 0, 2, J13);
	gsl_matrix_set(Jacobian, 0, 3, J14);
	gsl_matrix_set(Jacobian, 0, 4, J15);
	gsl_matrix_set(Jacobian, 0, 5, J16);

	// baris kedua
	gsl_matrix_set(Jacobian, 1, 0, J21);
	gsl_matrix_set(Jacobian, 1, 1, J22);
	gsl_matrix_set(Jacobian, 1, 2, J23);
	gsl_matrix_set(Jacobian, 1, 3, J24);
	gsl_matrix_set(Jacobian, 1, 4, J25);
	gsl_matrix_set(Jacobian, 1, 5, J26);

	// baris ketiga
	gsl_matrix_set(Jacobian, 2, 0, J31);
	gsl_matrix_set(Jacobian, 2, 1, J32);
	gsl_matrix_set(Jacobian, 2, 2, J33);
	gsl_matrix_set(Jacobian, 2, 3, J34);
	gsl_matrix_set(Jacobian, 2, 4, J35);
	gsl_matrix_set(Jacobian, 2, 5, J36);

	printf("Jacobian Matrix = \n");

	for (int baris = 0; baris < Jacobian->size1; baris++){
		for (int kolom = 0; kolom < Jacobian->size2; kolom++){
			printf("%.3f ", gsl_matrix_get(Jacobian, baris, kolom));
		}
		printf("\n");
	}

	gsl_matrix *Jacobian_Inv;
	Jacobian_Inv = moore_penrose_pinv(Jacobian, rcond);

	printf("Inverse Jacobian Matrix = \n");

	for (int baris = 0; baris < Jacobian_Inv->size1; baris++){
		for (int kolom = 0; kolom < Jacobian_Inv->size2; kolom++){
			printf("%.3f ", gsl_matrix_get(Jacobian_Inv, baris, kolom));
		}
		printf("\n");
	}

	// mencari q dot
	float q_dot[6];
	float d_dot[3] = {x_dot, y_dot, z_dot};

	printf("size1 = %d | size2 = %d\n", Jacobian_Inv->size1, Jacobian_Inv->size2);

	printf("\n\nq_dot =\n");
	for (int baris = 0; baris < Jacobian_Inv->size1; baris++){
		q_dot[baris] = (gsl_matrix_get(Jacobian_Inv, baris, 0) * x_dot) + (gsl_matrix_get(Jacobian_Inv, baris, 1) * y_dot) + (gsl_matrix_get(Jacobian_Inv, baris, 2) * z_dot);
		printf ("%.3f", q_dot[baris]);
		printf("\n");
	}

	inv_jacob_result.q1 = q_dot[0] * RTD;
	inv_jacob_result.q2 = q_dot[1] * RTD;
	inv_jacob_result.q3 = q_dot[2] * RTD;
	inv_jacob_result.q4 = q_dot[3] * RTD;
	inv_jacob_result.q5 = q_dot[4] * RTD;
	inv_jacob_result.q6 = q_dot[5] * RTD;

	gsl_matrix_free(Jacobian);
	gsl_matrix_free(Jacobian_Inv);
	
	return inv_jacob_result;
}

int set_arm_inv_Jacob_PD(bool gripper){
    // Calculate target joint angles from inverse Jacobian
    float target_q1 = arm_data_now.tetha1 + jacob_inverse.q1 * dt;
    float target_q2 = arm_data_now.tetha2 + jacob_inverse.q2 * dt;
    float target_q3 = arm_data_now.tetha3 + jacob_inverse.q3 * dt;
    float target_q4 = arm_data_now.tetha4 + jacob_inverse.q4 * dt;
    float target_q5 = arm_data_now.tetha5 + jacob_inverse.q5 * dt;
    float target_q6 = arm_data_now.tetha6 + jacob_inverse.q6 * dt;
    
    // Calculate position errors
    error_q1 = target_q1 - arm_data_now.tetha1;
    error_q2 = target_q2 - arm_data_now.tetha2;
    error_q3 = target_q3 - arm_data_now.tetha3;
    error_q4 = target_q4 - arm_data_now.tetha4;
    error_q5 = target_q5 - arm_data_now.tetha5;
    error_q6 = target_q6 - arm_data_now.tetha6;
    
    // Calculate error derivatives
    diff_error_q1 = (error_q1 - error_bef_q1) / dt;
    diff_error_q2 = (error_q2 - error_bef_q2) / dt;
    diff_error_q3 = (error_q3 - error_bef_q3) / dt;
    diff_error_q4 = (error_q4 - error_bef_q4) / dt;
    diff_error_q5 = (error_q5 - error_bef_q5) / dt;
    diff_error_q6 = (error_q6 - error_bef_q6) / dt;
    
    // PD Control: calculate control effort
    float control_q1 = Kp * error_q1 + Kd * diff_error_q1;
    float control_q2 = Kp * error_q2 + Kd * diff_error_q2;
    float control_q3 = Kp * error_q3 + Kd * diff_error_q3;
    float control_q4 = Kp * error_q4 + Kd * diff_error_q4;
    float control_q5 = Kp * error_q5 + Kd * diff_error_q5;
    float control_q6 = Kp * error_q6 + Kd * diff_error_q6;
    
    // Apply control effort to current angles
    float new_q1 = arm_data_now.tetha1 + control_q1;
    float new_q2 = arm_data_now.tetha2 + control_q2;
    float new_q3 = arm_data_now.tetha3 + control_q3;
    float new_q4 = arm_data_now.tetha4 + control_q4;
    float new_q5 = arm_data_now.tetha5 + control_q5;
    float new_q6 = arm_data_now.tetha6 + control_q6;
    
    // Store current errors for next iteration
    error_bef_q1 = error_q1;
    error_bef_q2 = error_q2;
    error_bef_q3 = error_q3;
    error_bef_q4 = error_q4;
    error_bef_q5 = error_q5;
    error_bef_q6 = error_q6;
    
    printf("PD Control Applied:\n");
    printf("  Errors: q1=%.3f q2=%.3f q3=%.3f q4=%.3f q5=%.3f q6=%.3f\n", 
           error_q1, error_q2, error_q3, error_q4, error_q5, error_q6);
    printf("  Controls: q1=%.3f q2=%.3f q3=%.3f q4=%.3f q5=%.3f q6=%.3f\n", 
           control_q1, control_q2, control_q3, control_q4, control_q5, control_q6);
    
    return set_Arm(new_q1, new_q2, new_q3, new_q4, new_q5, new_q6, gripper);
}

void init_robot()
{
        target_pos.x = xd;
        target_pos.y = yd;
        target_pos.z = zd;

        prev_pos.x = x_prev;
        prev_pos.y = y_prev;
        prev_pos.z = z_prev;
        
	// q1=0.0 * DTR;
	q2 = 45.0 * DTR;
	arm_data_now.tetha2 = 45.0;
	q5 = -45.0 * DTR;
	arm_data_now.tetha5 = -45;

	arm_data_now.d0 = D1;
	arm_data_now.a1 = A1;
	arm_data_now.a2 = A2;
	arm_data_now.a3 = A3;
	arm_data_now.d4 = D4;
	arm_data_now.a5 = A5;
	arm_data_now.d6 = D6;

}

void Retrieve_serial(void) {
  int retval=1, i,j,k,l;

  unsigned char sdata[3]; 
  unsigned char baca;
  
  
	i=1;

  while (i>0) {
    fcntl(fd, F_SETFL, FNDELAY); 
    i=read(fd, &baca, 1);
    if ((i==1) && (baca == 0xF5)) {
    	printf("masuk\n");
    	sdata[0]=baca;
    	while (i<3) {
    		  if (read(fd, &baca, 1)>0) {sdata[i]=baca; i++;}
    	}
   	  printf("terbaca %x  %x  %x \n",sdata[0],sdata[1],sdata[2]);
   	  q1=(sdata[1])*180.0/255.0*DTR;
   	  q2=(sdata[2])*180.0/255.0*DTR;
    }
  } 

}

void animate_with_PD(float xd, float yd, float zd) {
    static float x_prev = 0, y_prev = 0, z_prev = 0;
    static int first_run = 1;
    
    // Get current end-effector position
    struct global_odom pos = forwardKinematics();
    
    // Initialize previous values on first run
    if (first_run) {
        x_prev = pos.x;
        y_prev = pos.y;
        z_prev = pos.z;
        first_run = 0;
    }
    
    // Position error (task space)
    float ex = xd - pos.x;
    float ey = yd - pos.y;
    float ez = zd - pos.z;
    
    // Numerical velocity estimation
    float x_dot = (pos.x - x_prev) / dt;
    float y_dot = (pos.y - y_prev) / dt;
    float z_dot = (pos.z - z_prev) / dt;
    
    // PD control in task space
    float vx_cmd = Kp * ex - Kd * x_dot;
    float vy_cmd = Kp * ey - Kd * y_dot;
    float vz_cmd = Kp * ez - Kd * z_dot;
    
    // Store for next iteration
    x_prev = pos.x;
    y_prev = pos.y;
    z_prev = pos.z;
    
    printf("\nTask Space Control:\n");
    printf("  Current: x=%.3f y=%.3f z=%.3f\n", pos.x, pos.y, pos.z);
    printf("  Target:  x=%.3f y=%.3f z=%.3f\n", xd, yd, zd);
    printf("  Error:   ex=%.3f ey=%.3f ez=%.3f\n", ex, ey, ez);
    printf("  Vel Cmd: vx=%.3f vy=%.3f vz=%.3f\n", vx_cmd, vy_cmd, vz_cmd);
    
    // Map task velocity to joint velocity using inverse Jacobian
    jacob_inverse = inverseJacobian(vx_cmd, vy_cmd, vz_cmd);
    
    // Apply joint-level PD control
    set_arm_inv_Jacob_PD(arm_data_now.gripper_state);
}
