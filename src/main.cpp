#include <iostream>
#include <thread>
#include <chrono>

// openGL
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <AntTweakBar.h>

#include "Shader.h"

// simulation
#include "ModelParameters.h"
#include "Scene.h"
#include "CompliantImplicitEuler.h"
#include "GUI.h"

// file path
const std::string filePath = "assets/sample-1.xml";
const std::string transPath = "assets/transform.dat";
const std::string vertexShader = "assets/TransformVertexShader.vertexshader";
const std::string fragmentShader = "assets/ColorFragmentShader.fragmentshader";

// scene parameter
ModelParameters* model;
Scene* scene;
CompliantImplicitEuler* stepper;

int numStrand, numParticle, numDof, numStep;
int currentStep = 0;
double t = 0;

// openGL data buffer
GLdouble* g_Dof_buffer_data;

// flag
bool isPause = true;
bool isSimulationEnd = false;
bool isEnd = false;

bool oneStep();
void autoStep();

// button callback function
void TW_CALL resetParameterCB(void*);
void TW_CALL startOrPauseCB(void* bar);
void TW_CALL stepCB(void*);
void TW_CALL exitCB(void*);

// AntTweatBar: GLFW -> GLFW3
inline void TwEventMouseButtonGLFW3(GLFWwindow* window, int button, int action, int mods)
{ TwEventMouseButtonGLFW(button, action); }
inline void TwEventMousePosGLFW3(GLFWwindow* window, double xpos, double ypos)
{ TwMouseMotion(int(xpos), int(ypos)); }
inline void TwEventMouseWheelGLFW3(GLFWwindow* window, double xoffset, double yoffset)
{ TwEventMouseWheelGLFW(yoffset); }
inline void TwEventKeyGLFW3(GLFWwindow* window, int key, int scancode, int action, int mods)
{ TwEventKeyGLFW(key, action); }
inline void TwEventCharGLFW3(GLFWwindow* window, int codepoint)
{ TwEventCharGLFW(codepoint, GLFW_PRESS); }
inline void TwWindowSizeGLFW3(GLFWwindow* window, int width, int height)
{ TwWindowSize(width, height); }


int main(){
	/********************** Initialize scene ************************/
    model = new ModelParameters(filePath, transPath);
    scene = new Scene(*model);
    stepper = new CompliantImplicitEuler(*scene, model->m_max_iters, model->m_criterion);

    numStrand = scene->getNumStrand();
    numParticle = scene->getNumParticle();
    numDof = scene->getNumDofs();
	numStep = int(model->m_duration / model->m_dt);

    std::cout << "Strands: " << numStrand
              << "\nTotal particle: " << numParticle << std::endl;

	/********************** Initialize GLFW *************************/
	if (!glfwInit())
	{
		std::cerr << "Failed to initialize GLFW\n";
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	GLFWwindow* window = glfwCreateWindow(1024, 768, "Hair-DER", NULL, NULL);
	if (window == NULL) {
		std::cerr << "Failed to open GLFW window.\n";
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent( window );

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	/*********************** Initialize AntTweakBar ********************/
	TwInit(TW_OPENGL_CORE, NULL);
	TwWindowSize(1024, 768);
	GUI gui(model, &t);

	// add button
	TwAddButton(gui.m_bar, "reset", resetParameterCB, gui.m_bar, "key=r");
	TwAddButton(gui.m_bar, "startOrPause", startOrPauseCB, gui.m_bar, "label='start' key=SPACE");
	TwAddButton(gui.m_bar, "step", stepCB, gui.m_bar, "key=s");
	TwAddButton(gui.m_bar, "quit", exitCB, gui.m_bar, "key=q");
	
	// Bind callback function
    glfwSetKeyCallback( window, (GLFWkeyfun)TwEventKeyGLFW3 );
	glfwSetMouseButtonCallback( window, (GLFWmousebuttonfun)TwEventMouseButtonGLFW3 );
	glfwSetCharCallback( window, (GLFWcharfun)TwEventCharGLFW3 );
	glfwSetCursorPosCallback( window, (GLFWcursorposfun)TwEventMousePosGLFW3 );
	glfwSetScrollCallback( window, (GLFWscrollfun)TwEventMouseWheelGLFW3 );

	/*********************** Initialize GLEW **************************/
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		std::cerr << "Failed to initialize GLEW\n";
		getchar();
		glfwTerminate();
		return -1;
	}

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile GLSL program from the shaders
	GLuint programID = LoadShaders(vertexShader.c_str(), fragmentShader.c_str());

	// Get a handle for "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
	glm::mat4 View = glm::lookAt(
		glm::vec3(0.5, 0.5, 0.5),    // Camera is at , in World Space
		glm::vec3(0, 0, 0),     	// and looks at the origin
		glm::vec3(0, 1, 0)      	// Head is up (set to 0,-1,0 to look upside-down)
	);
	glm::mat4 Model = glm::mat4(1.0f);
	glm::mat4 MVP = Projection * View * Model;
    
    // Create data array
	g_Dof_buffer_data = new GLdouble[numParticle * 3];
    for (int i = 0; i < numParticle * 3; ++i) {
        g_Dof_buffer_data[i] = model->m_rest_x(i);
    }

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLdouble) * 3 * numParticle, g_Dof_buffer_data, GL_DYNAMIC_DRAW);

	std::thread tid(autoStep);

	/************************* Main loop ***************************/
	while ( !isEnd && glfwWindowShouldClose(window) == 0 ) {
 		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glUseProgram(programID);

		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLdouble) * 3 * numParticle, g_Dof_buffer_data, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(
			0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_DOUBLE,          // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		// Draw the hair strand !
        for (int i = 0; i < numStrand; ++i) {
			glDrawArrays(
                GL_LINE_STRIP, 
                model->m_startIndex[i], 
                model->m_startIndex[i + 1] - model->m_startIndex[i]
            );
        }

		glDisableVertexAttribArray(0);

		TwDraw(); 

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

	};

	/************************** clear up and exit *************************/
	isEnd = true;

    delete model;
    delete scene;
    delete stepper;
    delete [] g_Dof_buffer_data;

	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteProgram(programID);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW nand AntTweakBar
	TwDraw(); 
	glfwTerminate();

	tid.join();

    return 0;
}

bool oneStep() {
	int i = 0, j = 0;

	if (currentStep == numStep)
		return true;

	if (stepper->stepScene( *scene, model->m_dt )) {
		stepper->accept( *scene, model->m_dt );

		// update vertex array
        for (int i = 0; i < numParticle; ++i) {
            Vector3s cp = scene->getX().segment( scene->getDof(i), 3 );
            for (j = 0;j < 3; ++j) {
                g_Dof_buffer_data[3 * i + j] = cp(j);
            }
        }

        if (++currentStep == numStep)
			isSimulationEnd = true;
		t += model->m_dt;
		scene->applyRigidTransform( t );
		
		return true;

	} else {
		std::cerr << "Fail to update at step: " << currentStep;
		return false;
	}
}

void autoStep() {
	while (!isEnd )
	{
		if (!isSimulationEnd && !isPause) {
			if (!oneStep()) 
				return;
			std::this_thread::sleep_for(std::chrono::microseconds(int(model->m_dt * 1e6)));
		}
	}
}

void TW_CALL resetParameterCB(void*) {
	if (isPause || isSimulationEnd) {
		// update parameters
		model->setStrandParameters();

		delete scene;
		delete stepper;

		scene = new Scene(*model);
		stepper = new CompliantImplicitEuler(*scene, model->m_max_iters, model->m_criterion);

		// update data array
		for (int i = 0; i < numParticle * 3; ++i) {
			g_Dof_buffer_data[i] = model->m_rest_x(i);
		}

		// clear flags
		t = 0;
		currentStep = 0;
		isPause = true;
		isSimulationEnd = false;

		std::cout << "===================== reset ===================\n";
	}
}

void TW_CALL startOrPauseCB(void* bar) {
	if (isPause) {
		TwSetParam((TwBar*)bar, "startOrPause", "label", TW_PARAM_CSTRING, 1, "pause");
		isPause = false;
	} else {
		TwSetParam((TwBar*)bar, "startOrPause", "label", TW_PARAM_CSTRING, 1, "start");
		isPause = true;
	}
}

void TW_CALL stepCB(void*) {
	if (isPause) oneStep();
}

void TW_CALL exitCB(void*) {
	isEnd = true;
}
