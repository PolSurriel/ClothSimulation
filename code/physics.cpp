#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <string>

/*
	Hecho por Pol Surriel Muixench
*/

// CONSTS
# define PI           3.14159265358979323846

const glm::vec3 BOX_TOP_LEFT_FRONT(5, 10, 5);
const glm::vec3 BOX_BOTTOM_RIGHT_BACK(-5, 0, -5);
const glm::vec3 BOX_SIZE(
	BOX_TOP_LEFT_FRONT.x - BOX_BOTTOM_RIGHT_BACK.x,
	BOX_TOP_LEFT_FRONT.y - BOX_BOTTOM_RIGHT_BACK.y,
	BOX_TOP_LEFT_FRONT.z - BOX_BOTTOM_RIGHT_BACK.z
);
const glm::vec3 BOX_CENTER(
	BOX_TOP_LEFT_FRONT.x - BOX_SIZE.x / 2.f,
	BOX_TOP_LEFT_FRONT.y - BOX_SIZE.y / 2.f,
	BOX_TOP_LEFT_FRONT.z - BOX_SIZE.z / 2.f
);

///////// Forward declarations
extern void cleanupPrims();

namespace MeshConfig
{
	namespace Cloth
	{
		float desiredLength = 1.f;
		float distanceBetweenNodes = 1.f;
		glm::vec3 pointA;
		glm::vec3 pointB;

		glm::vec3 offsetPosition;

		glm::vec3 reg;

		bool fixedEdges = true;


		float struct_elasticity   = 1;
		float shear_elasticity   = 1;
		float bending_elasticity   = 1;

		float struct_damping   = 1;
		float shear_damping   = 1;
		float bending_damping   = 1;
		
	}

}

namespace LilSpheres {
	extern const int maxParticles;
	extern int firstParticleIdx;
	extern int particleCount;
	extern void updateParticles(int startIdx, int count, float* array_data);
}

namespace Sphere {
	float oldRadius;
	glm::vec3 oldPosition;

	float newRadius;
	glm::vec3 newPosition;

	extern void updateSphere(glm::vec3 pos, float radius);
	extern void cleanupSphere();

	void init() {
		newRadius = 1.f;
		newPosition = glm::vec3(0.f, 2.f, -2.f);
	}

	void update() {
		if (oldRadius != newRadius || oldPosition != newPosition) {
			updateSphere(newPosition, newRadius);

			oldRadius = newRadius;
			oldPosition = newPosition;
		}
	}
}

namespace ClothMesh {
	extern void setupClothMesh();
	extern void drawClothMesh();

	extern void updateClothMesh(float* array_data);
	extern void cleanupClothMesh();
}

namespace {

	static struct PhysParams {
		float min = 0.f;
		float max = 10.f;
	} p_pars;

	struct Plane {
		float a;
		float b;
		float c;
		float d;

		Plane(glm::vec3 point, glm::vec3 normalVector) {
			a = normalVector.x;
			b = normalVector.y;
			c = normalVector.z;

			/*
				a*x + b*y + c*z + d = 0
				d = - a*x - b*y - c*z
			*/
			d = -a * point.x - b * point.y - c * point.z;
		}

		Plane(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3) {
			glm::vec3 normalVector = glm::normalize(glm::cross(p2 - p1, p3 - p1));

			a = normalVector.x;
			b = normalVector.y;
			c = normalVector.z;

			/*
				a*x + b*y + c*z + d = 0
				d = - a*x - b*y - c*z
			*/

			d = -a * p1.x - b * p1.y - c * p1.z;
		}

		glm::vec3 getNormal() const {
			return glm::vec3(a, b, c);
		}
	};

	struct Particle {
		float lifeExpectancySeconds;
		float lifeCountSeconds;

		enum RigidBody { STATIC, DYNAMIC } rigidbody;

		glm::vec3 position;
		glm::vec3 velocity;
		glm::vec3 const_accelation;
		glm::vec3 accelation;

		// Solver Verlet
		glm::vec3 lastPosition;

		bool collidedLastFrame;

		void applyVerletSolver(float dt)
		{
			glm::vec3 newPosition = position + (position - lastPosition) + (accelation + const_accelation) * dt*dt;

			velocity = (newPosition - position) / dt;
			lastPosition = position;
			position = newPosition;
		}

		Particle(glm::vec3 _position, glm::vec3 _velocity, float _lifeExpectancySeconds, RigidBody _rigidbody) :
			position(_position), velocity(_velocity),
			lifeExpectancySeconds(_lifeExpectancySeconds),
			lifeCountSeconds(_lifeExpectancySeconds),
			rigidbody(_rigidbody), collidedLastFrame(false),
			lastPosition(_position)
		{}


		void sphereCollision(glm::vec3 velocity, float dt){
			
			float distanceToSphere = glm::distance(position, Sphere::newPosition);
			if (distanceToSphere < Sphere::newRadius) {
				
				//REACTION
				glm::vec3 sphereToLastPoint = (position - Sphere::newPosition);

				float a = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
				float b = 2 * (velocity.x*(sphereToLastPoint.x) + velocity.y*(sphereToLastPoint.y) + velocity.z*(sphereToLastPoint.z));
				float c = sphereToLastPoint.x * sphereToLastPoint.x + sphereToLastPoint.y * sphereToLastPoint.y + sphereToLastPoint.z * sphereToLastPoint.z - (Sphere::newRadius*Sphere::newRadius);

				// Con la ecuacion de 2o grado encontramos 2 puntos donde su distancia con el centro de la esfera es igual a su radio, el punto donde entra en dicho objecto y el punto donde sale.
				// Solo nos interesa el 1r punto, donde tiene que rebotar la particula
				float newDt = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);

				glm::vec3 intersectionPoint = lastPosition + newDt * velocity;
				glm::vec3 normalPlaneVector = glm::normalize(intersectionPoint - Sphere::newPosition);

				Plane targetPlane(intersectionPoint, normalPlaneVector);

				const float coeficiente_elasticidad = 0.01f;
				const float coeficiente_fregamiento = 0.6f;

				aplicarRebote(coeficiente_elasticidad, coeficiente_fregamiento, dt, targetPlane, normalPlaneVector);
				
			}

		}

		void aplicarRebote(const float coeficiente_elasticidad, const float coeficiente_fregamiento, float dt, Plane targetPlane, glm::vec3 normalPlaneVector)
		{
			
			// Calculamos la posicion rebotada
			position = position - (1 + coeficiente_elasticidad) * (glm::dot(normalPlaneVector, position) + targetPlane.d) * normalPlaneVector;

			// Dado que verlet utiliza el vector last -> pos debemos rebotar la anterior posicion para que no hayan comportamientos extranos
			// Al hacer esto existe una discrepancia entre la velocidad que queremos y el vector last-> post. Ya que debemos calcularlo con el coeficiente de fregamiento
			// Pero dado que para calcular esa velocidad necesitamos el vector last -> pos lo hacemos igualmente
			lastPosition = lastPosition - (1 + coeficiente_elasticidad) * (glm::dot(normalPlaneVector, lastPosition) + targetPlane.d) * normalPlaneVector;

			// Calculamos la velocidad con fregamiento.
			glm::vec3 reboundVelocity = (position - lastPosition) / dt;
			auto velocityNormal = (normalPlaneVector*reboundVelocity)* normalPlaneVector;
			auto velocityTangentPlane = reboundVelocity - velocityNormal;

			this->velocity = reboundVelocity - coeficiente_fregamiento * velocityTangentPlane;

			// Corregimos last en funcion de esa velocidad
			lastPosition = position - this->velocity*dt;
		}
		
		void boxCollision(const Plane& boxPlane, float dt)
		{
			glm::vec3 normalPlaneVector = boxPlane.getNormal();

			float r = glm::dot(normalPlaneVector, position) + boxPlane.d;

			// Todos los normales del plano apuntan al centro del cubo
			// Si r <= 0 significa que la particula esta tocando el plano o en direccion contraria al normal, es decir, fuera de la caja
			if (r <= 0)
			{
				const float coeficiente_elasticidad = 0.01f;
				const float coeficiente_fregamiento = 0.01f;

				aplicarRebote(coeficiente_elasticidad, coeficiente_fregamiento, dt, boxPlane, normalPlaneVector);

			}
		

		}

	};

	struct Swing {
		Particle *p1;
		Particle *p2;

		glm::vec3 force;

		float elasticity;
		float damping;
		float length;

		Swing(Particle *_p1, Particle *_p2) : p1(_p1), p2(_p2)
		{
			
			length = glm::distance(_p1->position, _p2->position) * MeshConfig::Cloth::desiredLength;
			
		}

		void update() {
			updateForce();
			applyForce();
		}


		void updateForce() {
			glm::vec3 p2ToP1 = p1->position - p2->position;
			
			float particleDistance = glm::length(p2ToP1);
			auto directionVector = glm::normalize(p2ToP1);

			//Aplicamos la formula de la fuerza del muelle
			//Solo utilizamos una fuerza porque es la misma en sentido opuesto y así hacemos la mitad de calculos.
			force = -(elasticity *(particleDistance - length) + damping * (p1->velocity - p2->velocity) * directionVector) * directionVector;
			
		}

		void applyForce() {
			p1->accelation += force;
			p2->accelation -= force;
		}

	};

	static struct ParticleSystem {

		Particle* fixedParticle1;
		Particle* fixedParticle2;
		
		int numParticles;

		glm::vec3 acceleration;

		float particleLifeExpectancySeconds;
		float particleLifeSecondsCount;

		std::vector<Particle> particles;

		// SWINGS
		std::vector<Swing> structuralSwings;
		std::vector<Swing> shearSwings;
		std::vector<Swing> bendingSwings;

		float structuralDamping;
		float shearDamping;
		float bendingDamping;

		enum EmissionMode { CASCADE, FOUNTAIN } emissionMode;

		// BOX
		Plane* boxPlanes[6];

		// DRAW
		std::vector<glm::vec3> positionsToDraw;

		void updateSwingForces() {
			for (Swing& s : structuralSwings)
				s.update();

			for (Swing& s : shearSwings) 
				s.update();

			for (Swing& s : bendingSwings)
				s.update();
			
		}

		void calculeSwingConnections() {
			
			int cellsPerRow = 14;
			float structuralElasticity = 0.1f;

			for (int i = 0; i < 18; i++) {
				for (int j = 0; j < 14; j++) {

					int rowIndex = i * cellsPerRow;

					// STRUCTURAL

					// HORIZONTAL SWING
					if (j < 13) {
						Swing structuralSwing(&particles[rowIndex + j], &particles[rowIndex + j + 1]);
						
						structuralSwing.elasticity = MeshConfig::Cloth::struct_elasticity;
						structuralSwing.damping = MeshConfig::Cloth::struct_damping;
						
						structuralSwings.push_back(structuralSwing);
						
					}

					// VERTICAL SWING
					if (i < 17) {
						Swing structuralSwing(&particles[rowIndex + j], &particles[rowIndex+cellsPerRow + j]);

						structuralSwing.elasticity = MeshConfig::Cloth::struct_elasticity;
						structuralSwing.damping = MeshConfig::Cloth::struct_damping;
						
						structuralSwings.push_back(structuralSwing);
					}



					// SHEAR

					if(j < 16)
					{
						if(i < 13)
						{
							Swing shearSwing(&particles[rowIndex + j], &particles[rowIndex + cellsPerRow + j + 1]);

							shearSwing.elasticity = MeshConfig::Cloth::shear_elasticity;
							shearSwing.damping = MeshConfig::Cloth::shear_damping;
							
							shearSwings.push_back(shearSwing);
						}
						
						if(i != 0)
						{
							Swing shearSwing(&particles[rowIndex + j], &particles[rowIndex - cellsPerRow + j + 1]);

							shearSwing.elasticity = MeshConfig::Cloth::shear_elasticity;
							shearSwing.damping = MeshConfig::Cloth::shear_damping;
							

							shearSwings.push_back(shearSwing);
							
						}
						
					}
					

					

					
				}
			}

			// BENDING
			
			for (int i = 0; i < 16; i++) {
				int rowIndex = i * cellsPerRow;
				Swing bendingSwing(&particles[rowIndex], &particles[rowIndex +  2]);

				bendingSwing.elasticity = MeshConfig::Cloth::bending_elasticity;
				bendingSwing.damping = MeshConfig::Cloth::bending_damping;
				
				bendingSwings.push_back(bendingSwing);
				
			}

			for (int j = 0; j < 12; j++) {
				Swing bendingSwing(&particles[j], &particles[j + 2]);

				bendingSwing.elasticity = MeshConfig::Cloth::bending_elasticity;
				bendingSwing.damping = MeshConfig::Cloth::bending_damping;
				
				bendingSwings.push_back(bendingSwing);
			}
			
		}

		void fillPositionsToDraw() {
			positionsToDraw.clear();

			for (int i = 0; i < particles.size(); i++) {
				positionsToDraw.push_back(particles[i].position);
			}
		}

		float randomFloat(float a, float b) {
			float random = ((float)rand()) / (float)RAND_MAX;
			float diff = b - a;
			float r = random * diff;
			return a + r;
		}

		float mapValue(float value, float start1, float end1, float start2, float end2)
		{
			return (((value - start1) / (end1 - start1)) * (end2 - start2)) + start2;
		}


		void generateParticles() {

			float horizontalMargin = 1.f / 14.f;

			glm::vec3 pA = MeshConfig::Cloth::pointA + MeshConfig::Cloth::offsetPosition-MeshConfig::Cloth::reg;
			glm::vec3 pB = MeshConfig::Cloth::pointB + MeshConfig::Cloth::offsetPosition-MeshConfig::Cloth::reg;

			pA.x = mapValue(pA.x, 0, 1, BOX_CENTER.x - BOX_SIZE.x * .499f, BOX_CENTER.x + BOX_SIZE.x * .499f);
			pA.y = mapValue(pA.y, 0, 1, BOX_CENTER.y - BOX_SIZE.y * .499f, BOX_CENTER.y + BOX_SIZE.y * .499f);
			pA.z = mapValue(pA.z, 0, 1, BOX_CENTER.z - BOX_SIZE.z * .499f, BOX_CENTER.z + BOX_SIZE.z * .499f);

			pB.x = mapValue(pB.x, 0, 1, BOX_CENTER.x - BOX_SIZE.x * .499f, BOX_CENTER.x + BOX_SIZE.x * .499f);
			pB.y = mapValue(pB.y, 0, 1, BOX_CENTER.y - BOX_SIZE.y * .499f, BOX_CENTER.y + BOX_SIZE.y * .499f);
			pB.z = mapValue(pB.z, 0, 1, BOX_CENTER.z - BOX_SIZE.z * .499f, BOX_CENTER.z + BOX_SIZE.z * .499f);

			glm::vec3 directionAB = (pB - pA);


			float distanceAB = glm::length(directionAB);
			directionAB = glm::normalize(directionAB);

			float particleDistance = horizontalMargin * distanceAB * MeshConfig::Cloth::distanceBetweenNodes;


			glm::vec3 v(0, 0, 0);
			glm::vec3 m(0, -particleDistance, 0);

			for (int i = 0; i < 18; i++) {
				for (int j = 0; j < 14; j++) {
					glm::vec3 position = pA + directionAB * (j*particleDistance);
					position += v;

					glm::vec3 velocity(0, 0, 0);

					Particle toAdd(position, velocity, particleLifeExpectancySeconds, Particle::RigidBody::DYNAMIC);

					toAdd.const_accelation = glm::vec3(0.f, -9.81f, 0.f);
					toAdd.accelation = glm::vec3(0.f, 0.f, 0.f);
				

					particles.push_back(toAdd);
				}

				v += m;
			}

			fixedParticle1 = &particles[0];
			fixedParticle2 = &particles[13];

			fixedParticle1->rigidbody = Particle::RigidBody::STATIC;
			fixedParticle2->rigidbody = Particle::RigidBody::STATIC;
			
		}

		
		void update()
		{
			if (MeshConfig::Cloth::fixedEdges)
			{
				fixedParticle1->rigidbody = Particle::RigidBody::STATIC;
				fixedParticle2->rigidbody = Particle::RigidBody::STATIC;

				glm::vec3 pA = MeshConfig::Cloth::pointA + MeshConfig::Cloth::offsetPosition - MeshConfig::Cloth::reg;
				glm::vec3 pB = MeshConfig::Cloth::pointB + MeshConfig::Cloth::offsetPosition - MeshConfig::Cloth::reg;

				pA.x = mapValue(pA.x, 0, 1, BOX_CENTER.x - BOX_SIZE.x * .499f, BOX_CENTER.x + BOX_SIZE.x * .499f);
				pA.y = mapValue(pA.y, 0, 1, BOX_CENTER.y - BOX_SIZE.y * .499f, BOX_CENTER.y + BOX_SIZE.y * .499f);
				pA.z = mapValue(pA.z, 0, 1, BOX_CENTER.z - BOX_SIZE.z * .499f, BOX_CENTER.z + BOX_SIZE.z * .499f);

				pB.x = mapValue(pB.x, 0, 1, BOX_CENTER.x - BOX_SIZE.x * .499f, BOX_CENTER.x + BOX_SIZE.x * .499f);
				pB.y = mapValue(pB.y, 0, 1, BOX_CENTER.y - BOX_SIZE.y * .499f, BOX_CENTER.y + BOX_SIZE.y * .499f);
				pB.z = mapValue(pB.z, 0, 1, BOX_CENTER.z - BOX_SIZE.z * .499f, BOX_CENTER.z + BOX_SIZE.z * .499f);

				
				fixedParticle1->position = pA;
				fixedParticle2->position = pB;
				
			}
			else
			{
				fixedParticle1->rigidbody = Particle::RigidBody::DYNAMIC;
				fixedParticle2->rigidbody = Particle::RigidBody::DYNAMIC;
			}
		}

		void eraseParticles() {
			// Elimino todas las particulas que tengan lifeCountSeconds <= 0
			particles.erase(std::remove_if(
				particles.begin(), particles.end(),
				[](const Particle& p) {
					return p.lifeCountSeconds <= 0;
				}
			), particles.end());
		}

		void setBoxPlanes(glm::vec3 topLeftFront, glm::vec3 bottomRightBack) {
			float boxWidth = topLeftFront.x - bottomRightBack.x;
			float boxHeight = topLeftFront.y - bottomRightBack.y;
			float boxLenght = topLeftFront.z - bottomRightBack.z;

			// 1 -> topLeftFront
			// 2
			glm::vec3 topRightFront = topLeftFront - glm::vec3(boxWidth, 0.0f, 0.0f);
			// 3
			glm::vec3 bottomLeftFront = topLeftFront - glm::vec3(0.0f, boxHeight, 0.0f);
			// 4
			glm::vec3 bottomRightFront = bottomLeftFront - glm::vec3(boxWidth, 0.0f, 0.0f);

			// 5 -> bottomRightBack
			// 6
			glm::vec3 bottomLeftBack = bottomRightBack + glm::vec3(boxWidth, 0.0f, 0.0f);
			// 7
			glm::vec3 topRightBack = bottomRightBack + glm::vec3(0.0f, boxHeight, 0.0f);
			// 8
			glm::vec3 topLeftBack = topRightBack + glm::vec3(boxWidth, 0.0f, 0.0f);

			//   8---------7
			//  /|        /|
			// / |       / |
			//1---------2  |
			//|  6------|--5
			//| /       | /
			//|/        |/
			//3---------4

			// FRONT
			boxPlanes[0] = new Plane(topLeftFront, bottomLeftFront, topRightFront);
			// LEFT
			boxPlanes[1] = new Plane(topLeftFront, topLeftBack, bottomLeftFront);
			// RIGHT
			boxPlanes[2] = new Plane(topRightFront, bottomRightFront, bottomRightBack);
			// UP
			boxPlanes[3] = new Plane(topLeftFront, topRightFront, topLeftBack);
			// DOWN
			boxPlanes[4] = new Plane(bottomLeftFront, bottomLeftBack, bottomRightFront);
			// BACK
			boxPlanes[5] = new Plane(topLeftBack, topRightBack, bottomLeftBack);
		}

		void setupParticles()
		{
			generateParticles();
			calculeSwingConnections();
		}

		void handleRubberyDeformation(Swing & swing)
		{

			// Paper:
			// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.84.1732&rep=rep1&type=pdf
			
			const float TC_SCALAR = 1.1f;

			float dist = glm::distance(swing.p1->position, swing.p2->position);
			float maxDist = swing.length * TC_SCALAR;

			if (dist > maxDist)
			{
				glm::vec3 p1ToP2 = swing.p2->position - swing.p1->position;
				p1ToP2 = glm::normalize(p1ToP2);
				glm::vec3 midPoint = swing.p1->position + p1ToP2 * (dist / 2.f);


				// ASUMIMOS QUE NO HAY DOS RB ESTATICOS CONTIGUOS
				// LOS CASOS MENCIONADOS SE REFIEREN A LA FIGURA 3 DEL PAPER
				if (swing.p1->rigidbody == Particle::RigidBody::STATIC || swing.p2->rigidbody == Particle::RigidBody::STATIC)
				{
					//Case B (figure 3)
					if (swing.p1->rigidbody == Particle::RigidBody::STATIC)
					{
						// Hay que corregir p2
						swing.p2->position = swing.p1->position + p1ToP2 * (maxDist);
					}
					else
					{
						// Hay que corregir p1
						swing.p1->position = swing.p2->position - p1ToP2 * (maxDist);
					}

				}
				else
				{
					//Case A (figure 3)
					swing.p1->position = midPoint - p1ToP2 * (maxDist / 2.f);
					swing.p2->position = midPoint + p1ToP2 * (maxDist / 2.f);

				}

			}
	
		}
		

		void handleRubberyDeformation()
		{
			
			int iterations = 5;

			for(int i = 0; i < iterations; i++)
			{
				for( auto &swing : structuralSwings )
				{
					handleRubberyDeformation(swing);
				}

				for (auto &swing : shearSwings)
				{
					handleRubberyDeformation(swing);
				}

			}
			
		}

	} s_PS;
}

bool isNotPaused = true;

void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate

		if (ImGui::Button("Reset")) {
			s_PS.structuralSwings.clear();
			s_PS.shearSwings.clear();
			s_PS.bendingSwings.clear();
			
			s_PS.particles.clear();
			s_PS.setupParticles();
		}
			
		if (ImGui::Button(isNotPaused ? "Pause" : "Play")) isNotPaused = !isNotPaused;

		// MeshConfig mode
		ImGui::Spacing();
		ImGui::Text("Mesh config");
		ImGui::Spacing();

		ImGui::SliderFloat3("Position A", &MeshConfig::Cloth::pointA.x, 0.f, 1.f);
		ImGui::SliderFloat3("Position B", &MeshConfig::Cloth::pointB.x, 0.f, 1.f);
		ImGui::SliderFloat3("Position Offset", &MeshConfig::Cloth::offsetPosition.x, 0.f, 1.f);


		ImGui::Checkbox("Fixed edges", &MeshConfig::Cloth::fixedEdges);

		ImGui::Spacing();
		ImGui::Text("Mesh config");

		ImGui::DragFloat("Initial res distance between nodes (scalar)", &MeshConfig::Cloth::distanceBetweenNodes);
		ImGui::DragFloat("Desired length (scalar)", &MeshConfig::Cloth::desiredLength);

		ImGui::Spacing();
		ImGui::Text("Swing config");

		ImGui::DragFloat("struct elasticity)", &MeshConfig::Cloth::struct_elasticity);
		ImGui::DragFloat("struct damping);", &MeshConfig::Cloth::struct_damping);

		ImGui::DragFloat("shear elasticity)", &MeshConfig::Cloth::shear_elasticity);
		ImGui::DragFloat("shear damping);", &MeshConfig::Cloth::shear_damping);

		ImGui::DragFloat("bending elasticity)", &MeshConfig::Cloth::bending_elasticity);
		ImGui::DragFloat("bending damping);", &MeshConfig::Cloth::bending_damping);
		

		// SPHERE
		ImGui::Spacing();
		ImGui::Text("Shepere");

		ImGui::SliderFloat("Sphere Radius", &Sphere::newRadius, 0.3f, 5.f);
		ImGui::SliderFloat3("Position", &Sphere::newPosition.x, BOX_CENTER.x - 10.f, BOX_CENTER.x + 12.f);

	}

	ImGui::End();
}

void PhysicsInit() {
	
	s_PS.acceleration = glm::vec3(0.f, -9.81f, 0.f);
	s_PS.setBoxPlanes(BOX_TOP_LEFT_FRONT, BOX_BOTTOM_RIGHT_BACK);

	MeshConfig::Cloth::pointA = glm::vec3(0.f, 0.7f, 0.5f);
	MeshConfig::Cloth::reg = glm::vec3(0.5f, 0.7f, 0.5f);
	MeshConfig::Cloth::offsetPosition = glm::vec3(0.5f, 0.7f, 0.5f);
	MeshConfig::Cloth::pointB = glm::vec3(0.5f, 0.7f, 0.5f);

	
	extern bool renderParticles; renderParticles = true;
	LilSpheres::firstParticleIdx = 0;

	extern bool renderSphere; renderSphere = true;
	Sphere::init();

	
	extern bool renderCloth; renderCloth = true;

	s_PS.setupParticles();

}

void PhysicsUpdate(float dt) {


	int iterations = 6;
	dt /= iterations;
	
	for(int i = 0; i < iterations; i++)
	{
		
		if (isNotPaused) {

			s_PS.update();
			s_PS.updateSwingForces();

			for (Particle &p : s_PS.particles) {

				if (p.rigidbody == Particle::RigidBody::STATIC)
					continue;

				p.applyVerletSolver(dt);

			}


			s_PS.handleRubberyDeformation();


			for (Particle &p : s_PS.particles) {

				if (p.rigidbody == Particle::RigidBody::STATIC)
					continue;

				p.accelation *= 0;
				p.sphereCollision(p.velocity, dt);
				for (Plane* &boxPlane : s_PS.boxPlanes)
					p.boxCollision(*boxPlane, dt);

			}
		}
		
	}
	

	Sphere::update();

	// Actualizamos las particulas para pintar en el render
	s_PS.fillPositionsToDraw();
	LilSpheres::particleCount = s_PS.positionsToDraw.size();
	if (s_PS.positionsToDraw.size() > 0) {
		LilSpheres::updateParticles(0, s_PS.positionsToDraw.size(), &(s_PS.positionsToDraw.data()->x));
		ClothMesh::updateClothMesh(&(s_PS.positionsToDraw.data()->x));
	}	

}

void PhysicsCleanup() {
	for (Plane* &p : s_PS.boxPlanes) {
		delete p;
	}
	cleanupPrims();
}