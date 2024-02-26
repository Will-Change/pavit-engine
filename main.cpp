// Windows includes (For Time, IO, etc.)
#include <windows.h>
#include <mmsystem.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <vector> 

// OpenGL includes
#include <GL/glew.h>
#include <GL/freeglut.h>

// Assimp includes
#include <assimp/cimport.h> 
#include <assimp/scene.h> 
#include <assimp/postprocess.h> 

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/vector_angle.hpp>
#include "glm/gtx/string_cast.hpp"

#include <cmath>
#include <map>

#include "utils/mesh.h"
#include "utils/shaders.h"
#include "utils/vbo.h"
#include "main.h"

#include <numeric>

#define MESH_BODY "models/newbody5.obj"
#define MESH_UPPER_ARM "models/point5.obj"
#define MESH_LOWER_ARM "models/newone3.obj"
#define MESH_HAND "models/newhand2.obj"
#define MESH_TARGET "models/model.obj"

using namespace std;

GLuint shaderProgramID;

unsigned int mesh_vao = 0;
int width = 1250;
int height = 1250;


GLfloat rotate_x = 0.0f;
GLfloat rotate_y = 0.0f;
GLfloat rotate_z = 0.0f;

float arm_length = 1.5f;
float forearm_length = 1.0f;
float total_arm_length = 2.5f;

glm::vec3 arm_position = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 translate_lower_arm1 = glm::vec3(-1.5f, 0.0f, 0.0f);
glm::vec3 translate_lower_arm2 = glm::vec3(-1.0f, 0.0f, 0.0f);
glm::vec3 translate_hand = glm::vec3(-1.0f, 0.0f, 0.0f);
glm::vec3 translate_end_of_chain = glm::vec3(-0.8f, 0.0f, 0.0f);

glm::vec3 rotation_shoulder = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 rotation_forearm = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 rotate_lower_arm2 = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 rotation_hand = glm::vec3(0.0f, 0.0f, 0.0f);

glm::vec3 targetPosition = glm::vec3(-4.0f, 0.0f, 0.0f);

ModelData mesh_data;

struct JointConstraints {
    glm::vec3 minAngles; // Minimum rotation angles in degrees for x, y, and z axes
    glm::vec3 maxAngles; // Maximum rotation angles in degrees for x, y, and z axes
};

// Adjusting constraints for the elbow if it rotates around the Y-axis
JointConstraints constraints[] = {
    // Shoulder constraints as before
    {{-60.0f, 0.0f, -90.0f}, {180.0f, 180.0f, 90.0f}}, // Assuming XYZ: Flexion/Extension, Abduction/Adduction, Internal/External Rotation

    // Elbow constraints adjusted for Y-axis flexion/extension
    {{0.0f, 0.0f, 0.0f}, {0.0f, 145.0f, 145.0f}}, // Assuming Y-axis for flexion/extension

    // Wrist (Hand) constraints as before
    {{-75.0f, -20.0f, -90.0f}, {75.0f, 55.0f, 90.0f}},
};


int frame_count = 0;
bool twod = false;

glm::mat4 modelUpperArm;
glm::mat4 modelLowerArm1;
glm::mat4 modelLowerArm2;
glm::mat4 modelHand;
glm::mat4 modelEndOfChain;

void loadUpperArm(glm::mat4& modelUpperArm, int matrix_location)
{
	mesh_data = generateObjectBufferMesh(MESH_UPPER_ARM, shaderProgramID);
	
	modelUpperArm = glm::mat4(1.0f);
	if(twod) {
	modelUpperArm = glm::translate(modelUpperArm, glm::vec3(1.0f, -1.0f, 0.0f));
	}
	modelUpperArm = glm::translate(modelUpperArm, arm_position);
	glm::quat quaternion = glm::quat(rotation_shoulder);
	modelUpperArm = modelUpperArm * glm::toMat4(quaternion);

	glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(modelUpperArm));
	glDrawArrays(GL_TRIANGLES, 0, mesh_data.mPointCount);
}

void loadLowerArm(glm::mat4& modelParent, glm::mat4& modelLowerArm, int matrix_location, glm::vec3 translate_lower, glm::vec3 rotate_lower) {
	mesh_data = generateObjectBufferMesh(MESH_LOWER_ARM, shaderProgramID);

	modelLowerArm = glm::mat4(1.0f);

	modelLowerArm = glm::translate(modelLowerArm, translate_lower);
	glm::quat quaternion = glm::quat(rotate_lower);
	modelLowerArm = modelLowerArm * glm::toMat4(quaternion);

	modelLowerArm = modelParent * modelLowerArm;

	glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(modelLowerArm));
	glDrawArrays(GL_TRIANGLES, 0, mesh_data.mPointCount);
}

void loadHand(glm::mat4& modelLowerArm, glm::mat4& modelHand, int matrix_location) {
	mesh_data = generateObjectBufferMesh(MESH_HAND, shaderProgramID);

	modelHand = glm::mat4(1.0f);
	modelEndOfChain = glm::mat4(1.0f);

	modelHand = glm::translate(modelHand, translate_hand);

	glm::quat quaternion = glm::quat(rotation_hand);
	modelHand = modelHand * glm::toMat4(quaternion);

	modelHand = modelLowerArm * modelHand;

	modelEndOfChain = glm::translate(modelEndOfChain, translate_end_of_chain);
	modelEndOfChain = modelHand * modelEndOfChain;

	glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(modelHand));
	glDrawArrays(GL_TRIANGLES, 0, mesh_data.mPointCount);

}

void loadBody(glm::mat4& modelBody, int matrix_location)
{
	mesh_data = generateObjectBufferMesh(MESH_BODY, shaderProgramID);

	modelBody = glm::mat4(1.0f);
	modelBody = glm::translate(modelBody, glm::vec3(1.0f, -1.0f, 0.0f));

	glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(modelBody));
	glDrawArrays(GL_TRIANGLES, 0, mesh_data.mPointCount);
}

void loadTarget(int matrix_location)
{
	mesh_data = generateObjectBufferMesh(MESH_TARGET, shaderProgramID);

	glm::mat4 modelTarget = glm::mat4(1.0f);

	modelTarget = glm::translate(modelTarget, targetPosition);
	modelTarget = glm::scale(modelTarget, glm::vec3(0.001f, 0.001f, 0.001f));

	glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(modelTarget));
	glDrawArrays(GL_TRIANGLES, 0, mesh_data.mPointCount);
}

glm::vec3 view_translate = glm::vec3(6.0f, -6.0, -6.0f);
void display() {

	glEnable(GL_DEPTH_TEST); 
	glDepthFunc(GL_LESS);
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(shaderProgramID);

	int matrix_location = glGetUniformLocation(shaderProgramID, "model");
	int view_mat_location = glGetUniformLocation(shaderProgramID, "view");
	int proj_mat_location = glGetUniformLocation(shaderProgramID, "proj");


	glm::mat4 view = glm::mat4(1.0f);
	glm::mat4 persp_proj = glm::perspective(45.0f, (float)width / (float)height, 0.1f, 1000.0f);

	float angle = glm::radians(45.0f); 
    float tilt = glm::radians(35.264f); 
    float zoom = -1.0f; 

	if (twod == true){
		view = glm::translate(view, glm::vec3(0, 0, -8));
	}

	if (twod == false){
		view = glm::rotate(view, tilt, glm::vec3(1, 0, 0));
		view = glm::rotate(view, angle, glm::vec3(0, 1, 0));
		view = glm::translate(view, glm::vec3(0, 0, zoom));
		view = glm::translate(view, view_translate);
	}
	
	glUniformMatrix4fv(proj_mat_location, 1, GL_FALSE, glm::value_ptr(persp_proj));
	glUniformMatrix4fv(view_mat_location, 1, GL_FALSE, glm::value_ptr(view));

	loadTarget(matrix_location);
	glm::mat4 modelBody;

	loadBody(modelBody, matrix_location);
	loadUpperArm(modelUpperArm, matrix_location);
	loadLowerArm(modelUpperArm, modelLowerArm1, matrix_location, translate_lower_arm1, rotation_forearm);
	loadHand(modelLowerArm1, modelHand, matrix_location);

	glutSwapBuffers();
}
int i = 0;


float clampAngle(float angle, float minAngle, float maxAngle) {
    // Ensure the angle is in the range [-180, 180] for comparison
    angle = fmod(angle + 180.0f, 360.0f) - 180.0f;
    return glm::clamp(angle, minAngle, maxAngle);
}

// Apply constraints to the calculated rotation adjustments
void applyConstraints(glm::vec3& rotation, int jointIndex) {
    rotation.x = clampAngle(rotation.x, constraints[jointIndex].minAngles.x, constraints[jointIndex].maxAngles.x);
    rotation.y = clampAngle(rotation.y, constraints[jointIndex].minAngles.y, constraints[jointIndex].maxAngles.y);
    rotation.z = clampAngle(rotation.z, constraints[jointIndex].minAngles.z, constraints[jointIndex].maxAngles.z);
}


void calculateRotationAngle(glm::vec3& rotation, int axis, glm::vec3& startTransform, glm::vec3& endTransform) {
    int effective_axis = (axis == 1) ? 2 : 0; 

    float distance = glm::distance(startTransform, endTransform);
    rotation[axis] = glm::acos((startTransform[effective_axis] - endTransform[effective_axis]) / distance);

    int sign_axis = (axis == 1) ? 0 : 1;
    if (startTransform[sign_axis] < endTransform[sign_axis]) {
        rotation[axis] *= -1.0f;
    }
}

void calculateCCD(int& current_frame, int link_count, glm::vec3* globalTransforms[], int axis, float& rotationAdjustment, glm::vec3 goalTransform) {
    glm::vec3 transformHand(modelHand[3]);
    glm::vec3 transformEndChain(modelEndOfChain[3]);
    glm::vec3 rotationEndChain;
    glm::vec3 rotationGoal;

    calculateRotationAngle(rotationGoal, axis, (*globalTransforms[current_frame]), goalTransform);
    calculateRotationAngle(rotationEndChain, axis, (*globalTransforms[current_frame]), transformEndChain);
    rotationAdjustment = rotationGoal[axis] - rotationEndChain[axis];
}

void incrementFrameNumber(int& frame_number, int number_of_links)
{
	frame_number++;
	if (frame_number == number_of_links) {
		frame_number = 0;
	}
}
bool animating = false;
int rotation_axis = 2;
int frame_number = 0;
void updateScene() {
	if (animating) {
		animateTarget();
	}
		float rotateBy;
		glm::vec3 handTransform(modelHand[3]);
		glm::vec3 lowerArmTransform1(modelLowerArm1[3]);
		glm::vec3 lowerArmTransform2(modelLowerArm2[3]);
		glm::vec3 upperArmTransform(modelUpperArm[3]);

		float targetRotations[3] = {0.0f};
		glm::vec3* linkGlobalTransforms[] = { &handTransform ,&lowerArmTransform1, &upperArmTransform };
		glm::vec3* linkLocalRotations[] = { &rotation_hand, &rotation_forearm, &rotation_shoulder };

		calculateCCD(frame_number, 3, linkGlobalTransforms,2, rotateBy, targetPosition);
		calculateCCD(frame_number, 4, linkGlobalTransforms, rotation_axis, rotateBy, targetPosition);

		if(twod){
		applyConstraints((*linkLocalRotations[frame_number]), frame_number); 
		}

		(*linkLocalRotations[frame_number])[rotation_axis] += rotateBy;

		if(!twod) {
			calculateCCD(frame_number, 4, linkGlobalTransforms, 1, rotateBy, targetPosition);
			(*linkLocalRotations[frame_number])[1] += rotateBy;
			(*linkLocalRotations[frame_number])[2] += rotateBy;
		}

		frame_number++;
		if (frame_number == (3)) {
			frame_number = 0;
		}	
	frame_count++;
	glutPostRedisplay();
}

glm::vec3 pathPoints[] = { glm::vec3(-4.0f,0,0), glm::vec3(0,4.0f,0), glm::vec3(4.0f,2.0f, 0.0f), glm::vec3(3.0f,-4.0f,0.0f),glm::vec3(-2.0f,1.0f,0.0f),glm::vec3(-5.0f,0,0) };
int currentPointIndex = 1;
float progressAlongCurve = 0.0f;
float verticalOffset = 0.0f;
float previousVerticalPosition = 0.0f;
float previousHorizontalPosition = 0.0f;
float horizontalDistance = 0.0f;
float verticalDistance = 0.0f;

void animateTarget() {
    float amplitudeOfWave = 7.0f; // Can be adjusted to increase or decrease the height of the waves
    float frequencyOfWave = 1.0f; // Can be adjusted to make the waves more frequent or less frequent

    verticalOffset = pow(progressAlongCurve, 2) / (2.0f * (pow(progressAlongCurve, 2) - progressAlongCurve) + 1.0f);

    horizontalDistance = (pathPoints[currentPointIndex].x - pathPoints[currentPointIndex - 1].x);
    verticalDistance = (pathPoints[currentPointIndex].y - pathPoints[currentPointIndex - 1].y);

    float waveMotion = amplitudeOfWave * sin(frequencyOfWave * progressAlongCurve * 2 * 3.14);

    if (pathPoints[currentPointIndex].y > pathPoints[currentPointIndex - 1].y) {
        targetPosition.y = verticalOffset * verticalDistance + pathPoints[currentPointIndex - 1].y + waveMotion;
    } else {
        targetPosition.y = (1 - verticalOffset) * -verticalDistance + pathPoints[currentPointIndex].y + waveMotion;
    }

    targetPosition.x -= 0.0015f * horizontalDistance;
    progressAlongCurve += 0.0015f;

    if (glm::distance(targetPosition, pathPoints[currentPointIndex]) < 0.005f) {
        targetPosition = pathPoints[currentPointIndex];
        currentPointIndex++;
        
        std::cout << "Point Reached" << std::endl;
        std::cout << targetPosition.x << " " << targetPosition.y << std::endl;
        if (currentPointIndex >= sizeof(pathPoints) / sizeof(pathPoints[0])) {
            animating = false; 
            currentPointIndex = 0; 
        }
        progressAlongCurve = 0.0f;
        verticalOffset = 0.0f;
    }
}

void init()
{
	shaderProgramID = CompileShaders();

}

// Placeholder code for the keypress
float translate_speed = 0.1f;
void keypress(unsigned char key, int x, int y) {
	if (key == 'w') {
		targetPosition.y += translate_speed;
	}
	if (key == 'a') {
		targetPosition.x -= translate_speed;
	}
	if (key == 's') {
		targetPosition.y -= translate_speed;
	}
	if (key == 'd') {
		targetPosition.x += translate_speed;
	}
	if (key == 'q') {
		targetPosition.z -= translate_speed;
	}
	if (key == 'e') {
		targetPosition.z += translate_speed;
	}
	if (key == 'r') {
		
		targetPosition = pathPoints[0];
		if (animating) {
			animating = false;
		}
		else {
			animating = true;
		}
	}
	if (key == 'b') {
		if (twod) twod = false;
		else twod = true;
	}
}

int main(int argc, char** argv) {


	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(width, height);
	glutCreateWindow("IK -  Assignment2");


	glutDisplayFunc(display);
	glutIdleFunc(updateScene);
	glutKeyboardFunc(keypress);

	// A call to glewInit() must be done after glut is initialized!
	GLenum res = glewInit();
	// Check for any errors
	if (res != GLEW_OK) {
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
		return 1;
	}
	// Set up your objects and shaders
	init();
	// Begin infinite event loop
	glutMainLoop();
	return 0;
}
