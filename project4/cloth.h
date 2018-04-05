#ifndef CLOTH_H
#define CLOTH_H

#include <vector>
#include "glm/glm.hpp"
#include <freeglut.h>

#define X_AXIS_3 glm::vec3(1.0f, 0.0f, 0.0f)
#define Y_AXIS_3 glm::vec3(0.0f, 1.0f, 0.0f)
#define Z_AXIS_3 glm::vec3(0.0f, 0.0f, 1.0f)
#define X_AXIS_4 glm::vec4(1.0f, 0.0f, 0.0f, 0.0f)
#define Y_AXIS_4 glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)
#define Z_AXIS_4 glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)
#define W_AXIS_4 glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)

#define PI 3.14159265f

//damping value, step size for RK4 method and iterations num for the spring controlling method
#define DAMPING_VALUE 0.0075f
#define STEP_SIZE 0.25f
#define SPRING_ITERATIONS_NUM 15
#define ANCHORS_CAPACITY 2

//struct for a single particle
struct Particle
{
	Particle(glm::vec3 pos)
	{
		position = pos;
		previousPos = glm::vec3(0.0f);
		acceleration = glm::vec3(0.0f);
		normal = glm::vec3(0.0f);
		isFixed = false;
		mass = 1.0f;
	}

	glm::vec3 position;
	glm::vec3 previousPos;
	glm::vec3 normal;
	glm::vec3 acceleration;
	bool isFixed;
	float mass;
};

//strcut for a "spring" between each pair of the neighboor particles
struct Spring
{
	Spring(Particle* particle1, Particle* particle2)
	{
		p1 = particle1;
		p2 = particle2;
		distance = glm::distance(p1->position, p2->position);
	}

	Particle* p1;
	Particle* p2;
	float distance;
};

//RK method for the particle's force calculation
void RK4integration(Particle* p);
//function that checks if spring constrains are not satisfied and corrects particles' positions if so
void DoesSpringSatisfiesConstraint(Spring* s);
//force function for Runge-Kutta integration method
glm::vec3 ForceFunction(float m, glm::vec3 a);

class Cloth
{
public:
	int widthCount;
	int heightCount;
	int activeAnchor;
	int anchors[ANCHORS_CAPACITY];
	int anchorsNum;
	std::vector<Particle*> particles;
	std::vector<Spring*> springs;

	Cloth(float width, float height, int widthParticleCount, int heightParticleCount)
	{
		widthCount = widthParticleCount;
		heightCount = heightParticleCount;
		particles.resize(widthCount * heightCount);
		activeAnchor = 0;
		anchorsNum = 0;
		Init(width, height);
	}

	void Init(float width, float height)
	{
		//calculating and pushing back initial positions of the particles
		for (int i = 0; i < widthCount; i++)
		{
			for (int j = 0; j < heightCount; j++)
			{
				particles[ConvertIndex(i, j)] = new Particle(glm::vec3(width * (i / (float)widthCount) * 0.5f, height * (j / (float)heightCount), width * (i / (float)widthCount) * 0.5f));
			}
		}

		//initialization of the all springs between each pair of the neighboor particles
		for (int i = 0; i < widthCount; i++)
		{
			for (int j = 0; j < heightCount; j++)
			{
				if (i < widthCount - 1)
				{
					springs.push_back(new Spring(particles[ConvertIndex(i, j)], particles[ConvertIndex(i + 1, j)]));
					if (i < widthCount - 2)
					{
						springs.push_back(new Spring(particles[ConvertIndex(i, j)], particles[ConvertIndex(i + 2, j)]));
					}
				}
				if (j < heightCount - 1)
				{
					springs.push_back(new Spring(particles[ConvertIndex(i, j)], particles[ConvertIndex(i, j + 1)]));
					if (j < heightCount - 2)
					{
						springs.push_back(new Spring(particles[ConvertIndex(i, j)], particles[ConvertIndex(i, j + 2)]));
					}
				}
				if (i < widthCount - 1 && j < heightCount - 1)
				{
					springs.push_back(new Spring(particles[ConvertIndex(i, j)], particles[ConvertIndex(i + 1, j + 1)]));
					if (i < widthCount - 2 && j < heightCount - 2)
						springs.push_back(new Spring(particles[ConvertIndex(i, j)], particles[ConvertIndex(i + 2, j + 2)]));
				}
				if (i < widthCount - 1 && j < heightCount - 1)
				{
					springs.push_back(new Spring(particles[ConvertIndex(i + 1, j)], particles[ConvertIndex(i, j + 1)]));
					if (i < widthCount - 2 && j < heightCount - 2)
						springs.push_back(new Spring(particles[ConvertIndex(i + 2, j)], particles[ConvertIndex(i, j + 2)]));
				}
			}
		}
	}

	//normal of the triangle, which connects three neighboor particles
	glm::vec3 CalcNormal(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3)
	{
		return glm::cross(v2 - v1, v3 - v1);
	}
	
	//calculating the force that will be apply to all three particles, that forms the triangle, based on the triangle's normal
	//we need this to apply wind force correctly
	glm::vec3 CalcTriangleForce(Particle *p1, Particle *p2, Particle *p3, glm::vec3 force)
	{
		glm::vec3 normal = CalcNormal(p1->position, p2->position, p3->position);
		return normal * glm::dot(force, glm::normalize(normal));
	}

	void Draw(GLenum mode, glm::mat4 cameraRotation)
	{
		std::vector<Particle*>::iterator particle;
		for (particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle)->normal = glm::vec3(0);
		}

		//calculating particles' normals, normal of the current particle is the sum of normals of the all triangles, which the current particle form
		//we need this for the nice shading of the cloth
		for (int i = 0; i < widthCount - 1; i++)
		{
			for (int j = 0; j < heightCount - 1; j++)
			{
				Particle* p1 = particles[ConvertIndex(i, j)];
				Particle* p2 = particles[ConvertIndex(i + 1, j)];
				Particle* p3 = particles[ConvertIndex(i, j + 1)];
				Particle* p4 = particles[ConvertIndex(i + 1, j + 1)];

				glm::vec3 normal = CalcNormal(p2->position, p1->position, p3->position);
				p2->normal += normal;
				p1->normal += normal;
				p3->normal += normal;

				normal = CalcNormal(p4->position, p2->position, p3->position);
				p4->normal += normal;
				p2->normal += normal;
				p3->normal += normal;
			}
		}

		//drawing spheres to display anchor particles
		GLfloat anchorParticleMaterialColor[] = { 1.0f, 0.9f, 0.9f, 1.0 };
		GLfloat activeAnchorParticleMaterialColor[] = { 1.0f, 0.1f, 0.1f, 1.0 };
		for (int i = 0; i < anchorsNum; i++)
		{
			glPushMatrix();
			glLoadMatrixf(glm::value_ptr(glm::translate(cameraRotation, particles[anchors[i]]->position)));
			if (i == activeAnchor)
			{
				glMaterialfv(GL_FRONT, GL_SPECULAR, activeAnchorParticleMaterialColor);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, activeAnchorParticleMaterialColor);
			}
			else
			{
				glMaterialfv(GL_FRONT, GL_SPECULAR, anchorParticleMaterialColor);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, anchorParticleMaterialColor);
			}
			glutSolidSphere(0.5, 20, 20);
			glPopMatrix();
		}

		glBegin(mode);
		GLfloat clothMaterialColor[] = { 0.2f, 0.4f, 0.4f, 1.0 };
		glMaterialfv(GL_FRONT, GL_SPECULAR, clothMaterialColor);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, clothMaterialColor);
		//draw loop
		for (int i = 0; i < widthCount - 1; i++)
		{
			for (int j = 0; j < heightCount - 1; j++)
			{
				Particle* p1 = particles[ConvertIndex(i, j)];
				Particle* p2 = particles[ConvertIndex(i + 1, j)];
				Particle* p3 = particles[ConvertIndex(i, j + 1)];
				Particle* p4 = particles[ConvertIndex(i + 1, j + 1)];

				glNormal3fv((GLfloat*)&glm::normalize(p2->normal));
				glVertex3fv((GLfloat*)&(p2->position));

				glNormal3fv((GLfloat*)&glm::normalize(p1->normal));
				glVertex3fv((GLfloat*)&(p1->position));

				glNormal3fv((GLfloat*)&glm::normalize(p3->normal));
				glVertex3fv((GLfloat*)&(p3->position));

				glNormal3fv((GLfloat*)&glm::normalize(p4->normal));
				glVertex3fv((GLfloat*)&(p4->position));

				glNormal3fv((GLfloat*)&glm::normalize(p2->normal));
				glVertex3fv((GLfloat*)&(p2->position));

				glNormal3fv((GLfloat*)&glm::normalize(p3->normal));
				glVertex3fv((GLfloat*)&(p3->position));
			}
		}
		glEnd();
	}

	//calculating new positions for the particles
	void UpdateParticlePositions()
	{
		std::vector<Spring*>::iterator spring;
		for (int i = 0; i < SPRING_ITERATIONS_NUM; i++)
		{
			for (spring = springs.begin(); spring != springs.end(); spring++)
			{
				//checking if spring constrains of the two current particles are not satisfied and corrects particles' positions if so
				DoesSpringSatisfiesConstraint(*spring);
			}
		}

		std::vector<Particle*>::iterator particle;
		for (particle = particles.begin(); particle != particles.end(); particle++)
		{
			//calculating new particles' positions using Runge-Kutta integration method
			RK4integration(*particle);
		}
	}

	//adding force to all particles independently or based on the triangles's normals  
	void AddForce(glm::vec3 force, bool calcNormal)
	{
		for (int i = 0; i < widthCount - 1; i++)
		{
			for (int j = 0; j < heightCount - 1; j++)
			{
				if (calcNormal)
				{
					Particle* p1 = particles[ConvertIndex(i, j)];
					Particle* p2 = particles[ConvertIndex(i + 1, j)];
					Particle* p3 = particles[ConvertIndex(i, j + 1)];
					Particle* p4 = particles[ConvertIndex(i + 1, j + 1)];

					glm::vec3 finalForce1 = CalcTriangleForce(p2, p1, p3, force);
					glm::vec3 finalForce2 = CalcTriangleForce(p4, p2, p3, force);

					p1->acceleration += finalForce1 / p1->mass;
					p2->acceleration += finalForce1 / p2->mass;
					p3->acceleration += finalForce1 / p3->mass;

					p4->acceleration += finalForce2 / p4->mass;
					p2->acceleration += finalForce2 / p4->mass;
					p3->acceleration += finalForce2 / p4->mass;
				}
				else
				{
					particles[ConvertIndex(i, j)]->acceleration += force / particles[ConvertIndex(i, j)]->mass;
				}

			}
		}
	}

	int ConvertIndex(int i, int j)
	{
		return i * widthCount + j;
	}

	void ChangeActiveAnchor()
	{
		if (++activeAnchor >= anchorsNum)
			activeAnchor = 0;
	}

	void AddAnchor(int i, int j)
	{
		if (anchorsNum < ANCHORS_CAPACITY)
		{
			 anchors[anchorsNum] = ConvertIndex(i, j);
			 particles[anchors[anchorsNum]]->isFixed = true;
			 anchorsNum++;
		}
	}
};

void RK4integration(Particle* p)
{
	if (!p->isFixed)
	{
		glm::vec3 oldPosition = p->position;

		glm::vec3 k1 = ForceFunction(p->mass, p->acceleration);
		glm::vec3 k2 = ForceFunction(p->mass + STEP_SIZE / 2.0f, p->acceleration + k1 * STEP_SIZE / 2.0f);
		glm::vec3 k3 = ForceFunction(p->mass + STEP_SIZE / 2.0f, p->acceleration + k2 * STEP_SIZE / 2.0f);
		glm::vec3 k4 = ForceFunction(p->mass + STEP_SIZE, p->acceleration + k3 * STEP_SIZE);
		p->position = p->position + (p->position - p->previousPos) * (1.0f - DAMPING_VALUE) + (STEP_SIZE / 6) * (k1 + k2 + k3 + k4);

		p->previousPos = oldPosition;
		p->acceleration = glm::vec3(0.0f, 0.0f, 0.0f);
	}
}

void DoesSpringSatisfiesConstraint(Spring* s)
{
	float currentDistance = glm::distance(s->p1->position, s->p2->position);
	glm::vec3 correctionVector = (s->p2->position - s->p1->position) * (1 - s->distance / currentDistance);

	if (!s->p1->isFixed)
		s->p1->position += correctionVector / 2.0f;

	if (!s->p2->isFixed)
		s->p2->position -= correctionVector / 2.0f;;
}

glm::vec3 ForceFunction(float m, glm::vec3 a)
{
	return m * a;
}

#endif // !CLOTH_H