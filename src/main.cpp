#include <iostream>

// openGL
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "OpenGL/Shader.h"

// simulation
#include "ModelParameters.h"
#include "Scene.h"
#include "CompliantImplicitEuler.h"

// stepper parameters
const double dt = 0.04;
const int max_iter = 50;
const double criterion = 1e-8;

const std::string filePath = "assets/sample.xml";
const std::string vertexShader = "src/OpenGL/TransformVertexShader.vertexshader";
const std::string fragmentShader = "src/OpenGL/ColorFragmentShader.fragmentshader";

// Initialize scene
ModelParameters* model;
Scene* scene;
CompliantImplicitEuler* stepper;

int numStep;
int numStrand, numParticle, numDof;

GLdouble* g_Dof_buffer_data;

bool isEnd = false;

void keyCallback( GLFWwindow* window, int key, int scancode, int action, int mods);

int main(){
	// Initialise GLFW
	if (!glfwInit())
	{
		std::cerr << "Failed to initialize GLFW\n";
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	// glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
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
    glfwSetKeyCallback( window, keyCallback );

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		std::cerr << "Failed to initialize GLEW\n";
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

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
		glm::vec3(50, 50, 50),    // Camera is at (4,3,-3), in World Space
		glm::vec3(0, 0, 0),     // and looks at the origin
		glm::vec3(0, 1, 0)      // Head is up (set to 0,-1,0 to look upside-down)
	);
	glm::mat4 Model = glm::mat4(1.0f);
	glm::mat4 MVP = Projection * View * Model;

    // Initialize scene
    model = new ModelParameters(filePath, dt);
    scene = new Scene(*model);
    stepper = new CompliantImplicitEuler(*scene, max_iter, criterion);

    numStep = (int) model->m_duration / dt;
    numStrand = scene->getNumStrand();
    numParticle = scene->getNumParticle();
    numDof = scene->getNumDofs();

    std::cout << "Strands: " << numStrand
              << "\nTotal particle: " << numParticle << std::endl;
    
    // Create data array
	g_Dof_buffer_data = new GLdouble[numParticle * 3];
    for (int i = 0; i < numParticle * 3; ++i) {
        g_Dof_buffer_data[i] = model->m_strands(i);
    }

    GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLdouble) * 3 * numParticle, g_Dof_buffer_data, GL_DYNAMIC_DRAW);

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
                model->m_startIndex[i] * 3, 
                ( model->m_startIndex[i + 1] - model->m_startIndex[i] ) * 3
            );
        }

		glDisableVertexAttribArray(0);

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

	};

    delete model;
    delete scene;
    delete stepper;
    delete [] g_Dof_buffer_data;

	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteProgram(programID);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

    return 0;
}


void keyCallback( GLFWwindow* window, int key, int scancode, int action, int mods) {
    static int currentStep = 0;
    int i = 0, j = 0;

    if (key == GLFW_KEY_S && action == GLFW_PRESS && currentStep < numStep) {
        if (stepper->stepScene(*scene, dt)) {
            stepper->accept(*scene, dt);
            // update vertex array
            for (int i = 0; i < numParticle; ++i) {
                Vector3s cp = scene->getX().segment( scene->getDof(i), 3 );
                for (j = 0;j < 3; ++j) {
                    g_Dof_buffer_data[3 * i + j] = cp(j);
                }
            }
            ++currentStep;
        }
        else {
            std::cerr << "Update fail at step " << currentStep;
            isEnd = true;
        }
    } 
    else if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        isEnd = true;
    }
}