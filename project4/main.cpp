#include <stdio.h>
#include <time.h>
#include <iostream>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"
#include "glm/gtx/vector_angle.hpp"

#include <freeglut.h>
#include <math.h>
#include <vector>

#include "cloth.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800

//defines angles are in degrees
#define CAMERA_STEP 0.1f
#define ANCHER_STEP 1.0f
#define FORCE_STRENGTH_STEP 0.1f;
#define CAMERA_INITIAL_ANGLE_X_AXIS 0.0f
#define CAMERA_INITIAL_ANGLE_Y_AXIS 0.0f
#define CAMERA_INITIAL_ANGLE_Z_AXIS 0.0f

void Update(void);
void NextFrame(void);
void OnKeyUp(unsigned char c, int i, int j);
void OnKeyDown(unsigned char c, int i, int j);
float DegreesToRadians(float angle);
float RadiansToDegrees(float radians);

glm::vec3 cameraAngle = glm::vec3(DegreesToRadians(CAMERA_INITIAL_ANGLE_X_AXIS), DegreesToRadians(CAMERA_INITIAL_ANGLE_Y_AXIS), DegreesToRadians(CAMERA_INITIAL_ANGLE_Z_AXIS));
glm::vec3 cameraCenter = glm::vec3(0.0f, -2.0f, 0.0f);
float forcesStrength = 2.0f;
Cloth cloth(5.0f, 10.0f, 40, 40); //cloth object, 1200 particles
GLenum drawMode = GL_POINTS;

GLfloat lightAmbient[] = { 0.1, 0.1, 0.1, 1.0 };
GLfloat lightPosition[] = { 1.0, 0.0, 0.0, 1.0 };
GLfloat lightDiffuse[] = { 0.5, 0.5, 0.5, 1.0 };
GLfloat lightSpecular[] = { 0.5, 0.5, 0.5, 1.0 };

int main(int argc, char* argv[])
{
	srand(time(0));
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Project 4");
	glutDisplayFunc(Update);
	glutIdleFunc(NextFrame);

	glutKeyboardUpFunc(OnKeyUp);
	glutKeyboardFunc(OnKeyDown);

	glShadeModel(GL_SMOOTH);

	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glPointSize(2.0f);

	//adding two anchors
	cloth.AddAnchor(0, 0);
	cloth.AddAnchor(39, 0);

	glutMainLoop();

	return 0;
}

void Update(void)
{
	//adding force values (gravity and wind) to the cloth object
	cloth.AddForce(glm::vec3(0.0f, 0.2f, 0.0f) * STEP_SIZE, false);
	cloth.AddForce(glm::vec3(0.5f, 0.0f, -0.5f) * forcesStrength * STEP_SIZE, true);
	//calculatng new positions of the cloth's particles based on the new force values
	cloth.UpdateParticlePositions();

	glClearColor(0.0f, 0.5f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glm::mat4 perspectiveCameraMatrix = glm::perspective(80.0f, 1.0f, 1.0f, 5000.0f);

	glm::vec3 cameraEye = glm::vec3(0.0f, 0.0f, -15.0f);

	//camera controlled rotation
	glm::mat4 keyCameraRotation = glm::lookAt(cameraEye, cameraCenter, Y_AXIS_3);
	keyCameraRotation = glm::rotate(keyCameraRotation, cameraAngle.x, X_AXIS_3);
	keyCameraRotation = glm::rotate(keyCameraRotation, cameraAngle.y, Y_AXIS_3);
	keyCameraRotation = glm::rotate(keyCameraRotation, cameraAngle.z, Z_AXIS_3);

	//setting camera projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(glm::value_ptr(perspectiveCameraMatrix));

	//setting model matrix that is just lookAt matrix as soon as we are not applying transformations to the model matrix (so it's identity)
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(glm::value_ptr(keyCameraRotation));

	//drawing the cloth
	cloth.Draw(drawMode, keyCameraRotation);

	glutSwapBuffers();
}

void NextFrame(void)
{
	glutPostRedisplay();
}

void OnKeyUp(unsigned char c, int i, int j)
{
	switch (c)
	{
	//forcesStrength controls
	case '1':
		forcesStrength -= FORCE_STRENGTH_STEP;
		if (forcesStrength < 0.0f)
			forcesStrength = 0.0f;
		break;
	case '2':
		forcesStrength += FORCE_STRENGTH_STEP;
		break;
	//draw modes controls
	case 'm':
		if (drawMode == GL_POINTS)
			drawMode = GL_LINES;
		else if (drawMode == GL_LINES)
			drawMode = GL_TRIANGLES;
		else
			drawMode = GL_POINTS;
		break;
	case 'z':
		cloth.ChangeActiveAnchor();
		break;
	default:
		break;
	}
}

void OnKeyDown(unsigned char c, int i, int j)
{
	switch (c)
	{
	//camera movement controls
	case 'q':
		cameraCenter.x += CAMERA_STEP;
		break;
	case 'e':
		cameraCenter.x -= CAMERA_STEP;
		break;
	case 'r':
		cameraCenter.y += CAMERA_STEP;
		break;
	case 'f':
		cameraCenter.y -= CAMERA_STEP;
		break;
	//anchor particles movement controls
	case 'w':
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->isFixed = false;
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->position.y -= ANCHER_STEP;
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->isFixed = true;
		break;
	case 'a':
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->isFixed = false;
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->position.x -= ANCHER_STEP;
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->isFixed = true;
		break;
	case 's':
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->isFixed = false;
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->position.y += ANCHER_STEP;
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->isFixed = true;
		break;
	case 'd':
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->isFixed = false;
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->position.x += ANCHER_STEP;
		cloth.particles[cloth.anchors[cloth.activeAnchor]]->isFixed = true;
		break;
	default:
		break;
	}
}

float DegreesToRadians(float angle)
{
	return angle * PI / 180.0f;
}

float RadiansToDegrees(float radians)
{
	return radians * 180.0f / PI;
}