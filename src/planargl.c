/////////////////////////////////////////////////////////////
/*
 *  6 DOF Planar Robot Simulation using OpenGL
 *
 *  Originally developed from an OpenGL robotics template
 *  used in robotics coursework at the Department of
 *  Electrical Engineering.
 *
 *  Special acknowledgement to:
 *      Dr. Abdul Muis, M.Eng.
 *      Autonomous Control Electronics (ACONICS) Research Group
 *      https://www.ee.ui.ac.id/aconics
 *
 *  This version has been extended and modified to include:
 *      - 6 DOF planar manipulator model
 *      - Analytical Jacobian computation
 *      - Moore–Penrose pseudoinverse
 *      - Task-space and joint-space PD control
 *
 *  Author: Muhammad Sheva Al Fattah
 */
/////////////////////////////////////////////////////////////

#include <stdio.h> 
#include <stdlib.h> 
#include <GL/glut.h> // Header File For The GLUT Library
#include <GL/gl.h> // Header File For The OpenGL32 Library
#include <GL/glu.h> // Header File For The GLu32 Library
#include <unistd.h> // Header file for sleeping.
#include <math.h> 
#include <fcntl.h>			/* File control definitions */
#include <errno.h>			/* Error number definitions */
#include <termios.h>		/* POSIX terminal control definitions */
#include <sys/time.h>
#include "planar.c"

/* ascii code for the escape key */
#define ESCkey	27

/* The number/handle of our GLUT window */
int window, wcam;  

/* To draw a quadric model */
GLUquadricObj *obj;

// // ROBOT MODEL PARAMATER
#define Xoffset	0.0	
#define Yoffset	0.0
#define Zoffset	0.3

#define Link1 L1
#define Link2 L2

float *tetha1=&q1;
float *tetha2=&q2;
float *tetha3=&q3;
float *tetha4=&q4;
float *tetha5=&q5;
float *tetha6=&q6;

// float *x=&objx;
// float *y=&objy;

// char debug=0;

void Sim_main(void); // Deklarasi lebih awal agar bisa diakses oleh fungsi sebelumnya
void display(void); // fungsi untuk menampilkan gambar robot / tampilan camera awal

bool toggle_angle_control = false;
bool toggle_pd_control = false;  // NEW: Toggle untuk PD control mode
bool toggle_task_space_control = false;  // NEW: Toggle untuk task space control

// NEW: Target position for task space control
float target_x = 0.3;
float target_y = 0.0;
float target_z = 0.3;

/* define color */  
GLfloat green1[4]  ={0.8, 1.0, 0.8, 1.0};
GLfloat blue1[4]  ={0.1, 0.1, 1.0, 1.0};
GLfloat blue2[4]  ={0.2, 0.2, 1.0, 1.0};
GLfloat blue3[4]  ={0.3, 0.3, 1.0, 1.0};
GLfloat yellow1[4]={0.1, 0.1, 0.0, 1.0};
GLfloat yellow2[4]={0.2, 0.2, 0.0, 1.0};
GLfloat pink6[4] ={0.8, 0.55, 0.6, 1.0};
GLfloat yellow5[4]={0.8, 0.8, 0.0, 1.0};
GLfloat abu2[4]={0.5,0.5,0.5,1.0};
GLfloat gray1[4]  ={0.1, 0.1, 0.1, 1.0};
GLfloat gray2[4]  ={0.2, 0.2, 0.2, 1.0};
GLfloat gray3[4]  ={0.3, 0.3, 0.3, 1.0};
GLfloat gray4[4]  ={0.4, 0.4, 0.4, 1.0};
GLfloat gray5[4]  ={0.5, 0.5, 0.5, 1.0};
GLfloat gray6[4]  ={0.6, 0.6, 0.6, 1.0};
GLfloat gray7[4]  ={0.7, 0.7, 0.7, 1.0};
GLfloat gray8[4]  ={0.8, 0.8, 0.7, 1.0};
GLfloat gray9[4]  ={0.9, 0.9, 0.7, 1.0};
GLfloat red1[4]    = {1.0, 0.0, 0.0, 1.0};   // merah cerah
GLfloat orange1[4] = {1.0, 0.5, 0.0, 1.0};   // oranye cerah


void  drawOneLine(double x1, double y1, double x2, double y2) 
   {glBegin(GL_LINES); glVertex3f((x1),(y1),0.0); glVertex3f((x2),(y2),0.0); glEnd();}
   
void  model_cylinder(GLUquadricObj * object, GLdouble lowerRadius,
  GLdouble upperRadius, GLdouble length, GLint res, GLfloat *color1, GLfloat *color2)
{
  glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glTranslatef(0,0,-length/2);
	  gluCylinder(object, lowerRadius, upperRadius, length, 20, res);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    gluDisk(object, 0.01, lowerRadius, 20, res); 
    glTranslatef(0, 0, length);
    gluDisk(object, 0.01, upperRadius, 20, res); 
  glPopMatrix();
}

void  model_box(GLfloat width, GLfloat depth, GLfloat height, GLfloat *color1, GLfloat *color2, GLfloat *color3, int color)
{
   width=width/2.0;depth=depth/2.0;height=height/2.0;
   glBegin(GL_QUADS);
// top
    if (color==1) 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth, height);
    glVertex3f( width,-depth, height);
    glVertex3f( width, depth, height);
    glVertex3f(-width, depth, height);
   glEnd();
   glBegin(GL_QUADS);
// bottom
    if (color==1) 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth,-height);
    glVertex3f( width,-depth,-height);
    glVertex3f( width, depth,-height);
    glVertex3f(-width, depth,-height);
   glEnd();
   glBegin(GL_QUAD_STRIP);
// sides
    if (color==1) 
	    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    glVertex3f(-width,-depth,height);
    glVertex3f(-width,-depth,-height);
    glVertex3f(width,-depth,height);
    glVertex3f(width,-depth,-height);
    glVertex3f(width,depth,height);
    glVertex3f(width,depth,-height);
    glVertex3f(-width,depth,height);
    glVertex3f(-width,depth,-height);
    glVertex3f(-width,-depth,height);
   glEnd();
}

void disp_floor(void)
{
  int i,j,flagc=1;

  glPushMatrix();
  
  GLfloat dx=4.5,dy=4.5;
  GLint amount=15;
  GLfloat x_min=-dx/2.0, x_max=dx/2.0, x_sp=(GLfloat) dx/amount, y_min=-dy/2.0, y_max=dy/2.0, y_sp=(GLfloat) dy/amount;

  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1);
  for(i = 0; i<=48; i++){
     drawOneLine(-2.4+0.1*i, -2.4,       -2.4+0.1*i,  2.4);
     drawOneLine(-2.4,       -2.4+0.1*i,  2.4,       -2.4+0.1*i);
  }

  glPopMatrix();
}

void  lighting(void)
{
	GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
	GLfloat light_diffuse[] =  {0.4, 0.4, 0.4, 1.0};
	GLfloat light_specular[] = {0.3, 0.3, 0.3, 1.0};
	GLfloat light_position[] = {2, 0.1, 7,1.0};
	GLfloat spot_direction[] = {0.0, -0.1, -1.0, 1.0};

	glClearColor(0.0, 0.0, 0.0, 0.0);     
  
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 40.0);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_direction);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 4);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
}

void disp_robot(void)
{
   glPushMatrix();
    model_box(0.5, 0.5, 0.05, gray8, gray7, gray6,1);
    glTranslatef(Xoffset, Yoffset, 0.05);
    
    // Draw base
    model_cylinder(obj, 0.15, 0.15, 0.05, 2, blue1, yellow2);
    // Menuju joint-1
    glTranslatef(0, 0, 0.03);
  
    // Link Pertama glPushMatrix();
    glPushMatrix();
    // Gambar D1+A1
    glTranslatef (0, 0, (D1 + A1) / 2);
    model_cylinder (obj, 0.03, 0.03, D1 + A1, 2, pink6, yellow2); 
    glPopMatrix();

    // Menuju joint-2
    glTranslatef(0, 0, D1 + A1);
    glRotatef (*tetha1* RTD, 0, 0, 1);
    glRotatef (*tetha2 * RTD - 180, 0, 1, 0);
    
    // Link Kedua
    glPushMatrix();
    // Gambar link2
    glRotatef (90, 0, 1, 0);
    glTranslatef (0, 0, A2 / 2);
    model_cylinder (obj, 0.03, 0.03, A2, 2, yellow5, yellow2);
    glPopMatrix();
    
    // Menuju joint-3
    glTranslatef (A2, 0, 0);
    glTranslatef (A3, 0, 0);
    glRotatef (*tetha3* RTD, 0, 1, 0);
    glPushMatrix();
    
    // gambar link 3
    glTranslatef(0,0, D4/2);
    model_cylinder(obj, 0.03, 0.03, D4, 2, blue1, yellow2);
    glPopMatrix();
    
    // menuju joint-4 
    glTranslatef (0, 0, (D4) - (D4/20));
    glRotatef (*tetha4* RTD, 0, 0, 1);
    glPushMatrix();

    // Gambar link4
    glTranslatef (0, 0, D4 / 20);
    model_cylinder (obj, 0.03, 0.03, D4 / 10, 2, green1, yellow2); 
    glPopMatrix();

    // menuju joint-5
    glTranslatef (0, 0, D4 / 20);
    glRotatef (*tetha5* RTD + 90, 0, 1, 0);
    glPushMatrix();

    // Gambar link5
    glTranslatef(0, 0, (A5 + 0.01) / 2);
    model_cylinder (obj, 0.03, 0.03, A5 + 0.01, 2, red1, yellow2); 
    glPopMatrix();

    // menuju joint-6 
    glTranslatef (0, 0, A5 + 0.01);
    glRotatef (*tetha6* RTD, 0, 0, 1);
    glPushMatrix();

    // Gambar link6
    glTranslatef(0, 0, (D6 / 2) / 2);
    model_cylinder (obj, 0.03, 0.03, D6 / 2, 2, orange1, yellow2);

    // Gambar End Effector
    glTranslatef(0, 0, D6 / 2);
    model_cylinder(obj, 0.013, 0.013, D6 / 2, 2, gray7, yellow2);
    glPopMatrix();

    glPopMatrix();
}

// Draw Object
void display(void)
{
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; 
   disp_floor();
   disp_robot();

   /* since window is double buffered, 
      Swap the front and back buffers (used in double buffering). */
   glutSwapBuffers() ; 
}

void Sim_main(void)
{
  unsigned long Xr=0,Yr=0, Xg=0,Yg=0, Xb=0,Yb=0;
  int Nr=0, Ng=0, Nb=0;
  static unsigned int Rx,Ry, Gx,Gy, Bx,By;
  unsigned int i,j,k;
  static int count=0;
  glutSetWindow(window);
  k++;
  t += dt;  // Use dt from planar.c
  
  end_effector = forwardKinematics();
  
  // NEW: Task space control with PD
  if (toggle_task_space_control) {
     animate_with_PD(target_x, target_y, target_z);
     printf("t=%.3f | TASK SPACE PD | Target:(%.3f,%.3f,%.3f) Current:(%.3f,%.3f,%.3f) | q1=%.3f q2=%.3f q3=%.3f q4=%.3f q5=%.3f q6=%.3f\n", 
            t, target_x, target_y, target_z, end_effector.x, end_effector.y, end_effector.z, 
            q1*RTD, q2*RTD, q3*RTD, q4*RTD, q5*RTD, q6*RTD);
  }
  else if (toggle_angle_control == false) {
     printf("t=%.3f | MANUAL | x=%.3f y=%.3f z=%.3f | q1=%.3f q2=%.3f q3=%.3f q4=%.3f q5=%.3f q6=%.3f\n", 
            t, end_effector.x, end_effector.y, end_effector.z, 
            q1*RTD, q2*RTD, q3*RTD, q4*RTD, q5*RTD, q6*RTD);
  }
  else {
     printf("t=%.3f | CONTROL | x=%.3f y=%.3f z=%.3f | q1=%.3f q2=%.3f q3=%.3f q4=%.3f q5=%.3f q6=%.3f\n", 
            t, end_effector.x, end_effector.y, end_effector.z, 
            q1*RTD, q2*RTD, q3*RTD, q4*RTD, q5*RTD, q6*RTD);
  }

  display();
  usleep(2000);
  display();
  usleep(2000);
}

void keyboard(unsigned char key, int i, int j)
{
   switch(key){
      case ESCkey: printf("\nTERTUTUP\n"); exit(1); break;
      case 'X': glRotatef(1, 1, 0, 0); break;
      case 'x': glRotatef(-1, 1, 0, 0); break;
      case 'Y': glRotatef(1, 0, 1, 0); break;
      case 'y': glRotatef(-1, 0, 1, 0); break;
      case 'Z': glRotatef(1, 0, 0, 1); break;
      case 'z': glRotatef(-1, 0, 0, 1); break;
      case 'a': glTranslatef(0, 0.1, 0); break;
      case 'd': glTranslatef(0, -0.1, 0); break;
      case 's': glTranslatef(-0.1, 0, 0); break;
      case 'w': glTranslatef(0.1, 0, 0); break;
      case 'r': glTranslatef(0, 0, -0.1); break;
      case 'f': glTranslatef(0, 0, 0.1); break;
      case 'p': glScalef(1.1, 1.1, 1.1); break;
      case 'o': glScalef(0.9, 0.9, 0.9); break;
      
      case '-':
         if (toggle_angle_control)
            toggle_angle_control = false;
         else
            toggle_angle_control = true;
         printf("Toggle Angle Control: %s\n", toggle_angle_control ? "ON" : "OFF");
         break;
         
      case '=':  
         if (toggle_pd_control)
            toggle_pd_control = false;
         else
            toggle_pd_control = true;
         printf("Toggle PD Control: %s\n", toggle_pd_control ? "ON" : "OFF");
         break;
         
      case 't':  
      case 'T':
         if (toggle_task_space_control) {
            toggle_task_space_control = false;
            printf("Task Space Control: OFF\n");
         }
         else {
            toggle_task_space_control = true;
            // Set current position as initial target
            end_effector = forwardKinematics();
            target_x = end_effector.x;
            target_y = end_effector.y;
            target_z = end_effector.z;
            printf("Task Space Control: ON (Target: %.3f, %.3f, %.3f)\n", target_x, target_y, target_z);
         }
         break;
         
      // NEW: Adjust target position in task space mode  
      case 'u': if(toggle_task_space_control) target_x += 0.01; break;  // X+
      case 'j': if(toggle_task_space_control) target_x -= 0.01; break;  // X-
      case 'i': if(toggle_task_space_control) target_y += 0.01; break;  // Y+
      case 'k': if(toggle_task_space_control) target_y -= 0.01; break;  // Y-
      case 'U': if(toggle_task_space_control) target_z += 0.01; break;  // Z+
      case 'J': if(toggle_task_space_control) target_z -= 0.01; break;  // Z-
      
      // Joint controls with conditional PD
      case '1': 
         if(toggle_angle_control == false)
            *tetha1 += 2.5 * DTR;
         else {
            // Set target, not current
            float target = (*tetha1 * RTD) + 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(target, q2 * RTD, q3 * RTD, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q1 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q1 PD Control Failed\n");
            }
            else {
               if (set_Arm(target, q2 * RTD, q3 * RTD, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q1 Success\n");
               else
                  printf("q1 Failed\n");
            }
         }
         break;
         
      case 33:  // !
         if (toggle_angle_control == false)
            *tetha1 -= 2.5 *DTR;
         else {
            float target = (*tetha1 * RTD) - 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(target, q2 * RTD, q3 * RTD, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q1 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q1 PD Control Failed\n");
            }
            else {
               if (set_Arm(target, q2 * RTD, q3 * RTD, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q1 Success\n");
               else
                  printf("q1 Failed\n");
            }
         }
         break;
         
      case '2':
         if (!toggle_angle_control)
            *tetha2 += 2.5 * DTR;
         else {
            float target = (*tetha2 * RTD) + 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, target, q3 * RTD, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q2 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q2 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, target, q3 * RTD, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q2 Success\n");
               else
                  printf("q2 Failed\n");
            }
         }
         break;

      case 64: // @
         if (!toggle_angle_control)
            *tetha2 -= 2.5 * DTR;
         else {
            float target = (*tetha2 * RTD) - 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, target, q3 * RTD, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q2 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q2 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, target, q3 * RTD, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q2 Success\n");
               else
                  printf("q2 Failed\n");
            }
         }
         break;
         
      case '3':
         if (!toggle_angle_control)
            *tetha3 += 2.5 * DTR;
         else {
            float target = (*tetha3 * RTD) + 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, q2 * RTD, target, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q3 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q3 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, q2 * RTD, target, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q3 Success\n");
               else
                  printf("q3 Failed\n");
            }
         }
         break;

      case 35: // #
         if (!toggle_angle_control)
            *tetha3 -= 2.5 * DTR;
         else {
            float target = (*tetha3 * RTD) - 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, q2 * RTD, target, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q3 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q3 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, q2 * RTD, target, q4 * RTD, q5 * RTD, q6 * RTD, true))
                  printf("q3 Success\n");
               else
                  printf("q3 Failed\n");
            }
         }
         break;

      case '4':
         if (!toggle_angle_control)
            *tetha4 += 2.5 * DTR;
         else {
            float target = (*tetha4 * RTD) + 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, q2 * RTD, q3 * RTD, target, q5 * RTD, q6 * RTD, true))
                  printf("q4 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q4 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, q2 * RTD, q3 * RTD, target, q5 * RTD, q6 * RTD, true))
                  printf("q4 Success\n");
               else
                  printf("q4 Failed\n");
            }
         }
         break;

      case 36: // $
         if (!toggle_angle_control)
            *tetha4 -= 2.5 * DTR;
         else {
            float target = (*tetha4 * RTD) - 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, q2 * RTD, q3 * RTD, target, q5 * RTD, q6 * RTD, true))
                  printf("q4 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q4 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, q2 * RTD, q3 * RTD, target, q5 * RTD, q6 * RTD, true))
                  printf("q4 Success\n");
               else
                  printf("q4 Failed\n");
            }
         }
         break;

      case '5':
         if (!toggle_angle_control)
            *tetha5 += 2.5 * DTR;
         else {
            float target = (*tetha5 * RTD) + 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, q2 * RTD, q3 * RTD, q4 * RTD, target, q6 * RTD, true))
                  printf("q5 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q5 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, q2 * RTD, q3 * RTD, q4 * RTD, target, q6 * RTD, true))
                  printf("q5 Success\n");
               else
                  printf("q5 Failed\n");
            }
         }
         break;

      case 37: // %
         if (!toggle_angle_control)
            *tetha5 -= 2.5 * DTR;
         else {
            float target = (*tetha5 * RTD) - 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, q2 * RTD, q3 * RTD, q4 * RTD, target, q6 * RTD, true))
                  printf("q5 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q5 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, q2 * RTD, q3 * RTD, q4 * RTD, target, q6 * RTD, true))
                  printf("q5 Success\n");
               else
                  printf("q5 Failed\n");
            }
         }
         break;

      case '6':
         if (!toggle_angle_control)
            *tetha6 += 2.5 * DTR;
         else {
            float target = (*tetha6 * RTD) + 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, q2 * RTD, q3 * RTD, q4 * RTD, q5 * RTD, target, true))
                  printf("q6 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q6 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, q2 * RTD, q3 * RTD, q4 * RTD, q5 * RTD, target, true))
                  printf("q6 Success\n");
               else
                  printf("q6 Failed\n");
            }
         }
         break;

      case 94: // ^
         if (!toggle_angle_control)
            *tetha6 -= 2.5 * DTR;
         else {
            float target = (*tetha6 * RTD) - 2.5;
            if (toggle_pd_control) {
               if (set_Arm_PD(q1 * RTD, q2 * RTD, q3 * RTD, q4 * RTD, q5 * RTD, target, true))
                  printf("q6 PD Control Success (target=%.2f)\n", target);
               else
                  printf("q6 PD Control Failed\n");
            } else {
               if (set_Arm(q1 * RTD, q2 * RTD, q3 * RTD, q4 * RTD, q5 * RTD, target, true))
                  printf("q6 Success\n");
               else
                  printf("q6 Failed\n");
            }
         }
         break;
         
      // Inverse Jacobian controls with PD
      case '7':
         jacob_inverse = inverseJacobian(0.01, 0, 0); // (x +1 cm)
         printf("InvJ dq1:%.3f dq2:%.3f dq3:%.3f dq4:%.3f dq5:%.3f dq6:%.3f\n", 
                jacob_inverse.q1, jacob_inverse.q2, jacob_inverse.q3, 
                jacob_inverse.q4, jacob_inverse.q5, jacob_inverse.q6);
         if (toggle_pd_control) {
            if (set_arm_inv_Jacob_PD(false))
               printf("Move Inv Jacob with PD Success\n");
            else
               printf("Move Inv Jacob with PD Failed\n");
         }
         else {
            if (set_arm_inv_Jacob(false))
               printf("Move Inv Jacob Success\n");
            else
               printf("Move Inv Jacob Failed\n");
         }
         break;
         
      case 38:  // &
         jacob_inverse = inverseJacobian(-0.01, 0, 0);
         printf("InvJ dq1:%.3f dq2:%.3f dq3:%.3f dq4:%.3f dq5:%.3f dq6:%.3f\n", 
                jacob_inverse.q1, jacob_inverse.q2, jacob_inverse.q3, 
                jacob_inverse.q4, jacob_inverse.q5, jacob_inverse.q6);
         if (toggle_pd_control) {
            if (set_arm_inv_Jacob_PD(false))
               printf("Move Inv Jacob with PD Success\n");
         }
         else {
            if (set_arm_inv_Jacob(false))
               printf("Move Inv Jacob Success\n");
         }
         break;
         
      case '8':
         jacob_inverse = inverseJacobian(0, 0.01, 0); // (y +1 cm)
         printf("InvJ dq1:%.3f dq2:%.3f dq3:%.3f dq4:%.3f dq5:%.3f dq6:%.3f\n", 
                jacob_inverse.q1, jacob_inverse.q2, jacob_inverse.q3, 
                jacob_inverse.q4, jacob_inverse.q5, jacob_inverse.q6);
         if (toggle_pd_control) {
            if (set_arm_inv_Jacob_PD(false))
               printf("Move Inv Jacob with PD Success\n");
         }
         else {
            if (set_arm_inv_Jacob(false))
               printf("Move Inv Jacob Success\n");
         }
         break;
         
      case 42:  // *
         jacob_inverse = inverseJacobian(0, -0.01, 0);
         printf("InvJ dq1:%.3f dq2:%.3f dq3:%.3f dq4:%.3f dq5:%.3f dq6:%.3f\n", 
                jacob_inverse.q1, jacob_inverse.q2, jacob_inverse.q3, 
                jacob_inverse.q4, jacob_inverse.q5, jacob_inverse.q6);
         if (toggle_pd_control) {
            if (set_arm_inv_Jacob_PD(false))
               printf("Move Inv Jacob with PD Success\n");
         }
         else {
            if (set_arm_inv_Jacob(false))
               printf("Move Inv Jacob Success\n");
         }
         break;
         
      case '9':
         jacob_inverse = inverseJacobian(0, 0, 0.01); // (z +1 cm)
         printf("InvJ dq1:%.3f dq2:%.3f dq3:%.3f dq4:%.3f dq5:%.3f dq6:%.3f\n", 
                jacob_inverse.q1, jacob_inverse.q2, jacob_inverse.q3, 
                jacob_inverse.q4, jacob_inverse.q5, jacob_inverse.q6);
         if (toggle_pd_control) {
            if (set_arm_inv_Jacob_PD(false))
               printf("Move Inv Jacob with PD Success\n");
         }
         else {
            if (set_arm_inv_Jacob(false))
               printf("Move Inv Jacob Success\n");
         }
         break;
         
      case 40:  // (
         jacob_inverse = inverseJacobian(0, 0, -0.01);
         printf("InvJ dq1:%.3f dq2:%.3f dq3:%.3f dq4:%.3f dq5:%.3f dq6:%.3f\n", 
                jacob_inverse.q1, jacob_inverse.q2, jacob_inverse.q3, 
                jacob_inverse.q4, jacob_inverse.q5, jacob_inverse.q6);
         if (toggle_pd_control) {
            if (set_arm_inv_Jacob_PD(false))
               printf("Move Inv Jacob with PD Success\n");
         }
         else {
            if (set_arm_inv_Jacob(false))
               printf("Move Inv Jacob Success\n");
         }
         break;
   }
}

void init(void) 
{ 
   obj = gluNewQuadric(); 
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
   glEnable(GL_DEPTH_TEST);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(40.0, 2, 0.2, 8);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt(0.2, -1.0, 1.5,  0.0, 0.2, 0.2,  0.0, 0.0, 1.0); 
   lighting();
   glShadeModel(GL_SMOOTH) ; 

   glutDisplayFunc (&display) ;
   glutKeyboardFunc(&keyboard);
}

// Main Program
int main(int argc, char** argv)
{
   glutInit (&argc, argv);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
   glutInitWindowSize(800,400);	
   glutInitWindowPosition (40, 100);
   window = glutCreateWindow ("6-DOF Robot Arm with PD Control");

   init() ;
   init_robot();

   printf("\n=== CONTROL KEYS ===\n");
   printf("'-' : Toggle Angle Control ON/OFF\n");
   printf("'=' : Toggle PD Control ON/OFF\n");
   printf("'t' : Toggle Task Space Control ON/OFF\n");
   printf("\n--- Joint Control (1-6) ---\n");
   printf("'1/!' : Joint 1 +/-\n");
   printf("'2/@' : Joint 2 +/-\n");
   printf("'3/#' : Joint 3 +/-\n");
   printf("'4/ : Joint 4 +/-\n");
   printf("'5/%%' : Joint 5 +/-\n");
   printf("'6/^' : Joint 6 +/-\n");
   printf("\n--- Inverse Jacobian (7-9) ---\n");
   printf("'7/&' : X +/- 1cm\n");
   printf("'8/*' : Y +/- 1cm\n");
   printf("'9/(' : Z +/- 1cm\n");
   printf("\n--- Task Space Target (when T mode ON) ---\n");
   printf("'u/j' : Target X +/-\n");
   printf("'i/k' : Target Y +/-\n");
   printf("'U/J' : Target Z +/-\n");
   printf("\n--- Camera View ---\n");
   printf("'w/s/a/d/r/f' : Translate view\n");
   printf("'X/Y/Z' (shift) : Rotate +1 deg\n");
   printf("'x/y/z' : Rotate -1 deg\n");
   printf("'p/o' : Zoom in/out\n");
   printf("ESC : Exit\n");
   printf("=====================\n\n");

   glutIdleFunc(&Sim_main);
   glutMainLoop () ;
   return 0 ;
}
