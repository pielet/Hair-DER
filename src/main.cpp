// #define OUTPUT_PNG

#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <AntTweakBar.h>
#include <png++/png.hpp>

#include "Shader.h"
#include "MathDefs.h"
#include "Simulator.h"
#include "Camera.h"

// file path
const std::string modelPath = "assets/sample-1.xml";
const std::string transPath = "assets/transform.dat";
const std::string vertexShader = "assets/TransformVertexShader.vertexshader";
const std::string fragmentShader = "assets/ColorFragmentShader.fragmentshader";
const std::string outputDir = "assets/output/";

// window size
const int width = 1024;
const int height = 768;

// AntTweatBar: GLFW -> GLFW3
inline void TwEventKeyGLFW3(GLFWwindow* window, int key, int scancode, int action, int mods)
{ TwEventKeyGLFW(key, action); }
inline void TwEventCharGLFW3(GLFWwindow* window, unsigned int codepoint)
{ TwEventCharGLFW(codepoint, GLFW_PRESS); }
inline void TwWindowSizeGLFW3(GLFWwindow* window, int width, int height)
{ TwWindowSize(width, height); }

// GLFW callback to Camera callback
inline void CameraMouseButton(GLFWwindow* window, int button, int action, int mods) { 
	Camera* cam = (Camera*) glfwGetWindowUserPointer(window);
	if (cam) cam->mouse_button_callback(button, action);
};
inline void CameraCursorPos(GLFWwindow* window, double xpos, double ypos) {
	Camera* cam = (Camera*) glfwGetWindowUserPointer(window);
	if (cam) cam->cursor_position_callback(xpos, ypos);
};
inline void CameraScroll(GLFWwindow* window, double xoffset, double yoffset) {
	Camera* cam = (Camera*) glfwGetWindowUserPointer(window);
	if (cam) cam->scroll_callback(yoffset); 
};

bool savePNG(const uint8_t* pixel, int step);

int main(){
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
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	GLFWwindow* window = glfwCreateWindow(width, height, "Hair-DER", NULL, NULL);
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
	TwWindowSize(width, height);

	/********************** Initialize scene ************************/
    Simulator sim(modelPath, transPath);

	Camera* camera = sim.getCamera();
	glfwSetWindowUserPointer(window, camera);

	int numParticle = sim.getNumParticle();
	int numStrand = sim.getNumStrand();
	const std::vector<int>& startIndex = sim.getStartIndex();

	std::cout << "Strands: " << numStrand
              << "\nTotal particle: " << numParticle << std::endl;
	
	// Bind callback function
    glfwSetKeyCallback( window, TwEventKeyGLFW3 );
	glfwSetMouseButtonCallback( window, CameraMouseButton);
	glfwSetCharCallback( window, TwEventCharGLFW3 );
	glfwSetCursorPosCallback( window, CameraCursorPos );
	glfwSetScrollCallback( window, CameraScroll );

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

	// prepare buffer & bind data
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	Matrix4s projection = camera->perspective(camera->radians(45.0), double(width) / height, 0.1, 100.0);
	Matrix4s view = camera->getLookAt();
	Matrix4s MVP = projection * view;
	
	GLfloat* g_Dof_buffer_data = sim.getBuffer();
	GLfloat* g_color_buffer_data = new GLfloat[3 * numParticle];
	for (int i = 0; i < numStrand; ++i) {
		Vector3f color = Vector3f::Random();
		for (int j = startIndex[i]; j < startIndex[i + 1]; ++j) {
			g_color_buffer_data[3 * j] = color(0);
			g_color_buffer_data[3 * j + 1] = color(1);
			g_color_buffer_data[3 * j + 2] = color(2);
		}
	}
	// grid data ...
	float interval = 0.2;
	int numGrid = 10;
	float range = interval * numGrid;
	GLfloat g_grid_data[2 * (numGrid * 2 + 1) * 12 + 12];
	Vector3f color;
	for (int i = 0; i < 2 * numGrid + 1; ++i) {
		if (i == numGrid) color = Vector3f(.6, .3, .3);
		else color = Vector3f(.25, .25, .25);
		float base = i * interval - range;
		int idx = 12 * i;
		g_grid_data[idx] = g_grid_data[idx + 6] = base;
		g_grid_data[idx + 1] = g_grid_data[idx + 7] = 0;
		g_grid_data[idx + 2] = -range;
		g_grid_data[idx + 8] = range;
		for (int j = 3; j < 6; ++j) {
			g_grid_data[idx + j] = g_grid_data[idx + 6 + j] = color(j - 3);
		}

		idx += (numGrid * 2 + 1) * 12;
		g_grid_data[idx] = -range;
		g_grid_data[idx + 6] = range;
		g_grid_data[idx + 1] = g_grid_data[idx + 7] = 0;
		g_grid_data[idx + 2] = g_grid_data[idx + 8] = base;
		for (int j = 3; j < 6; ++j) {
			g_grid_data[idx + j] = g_grid_data[idx + 6 + j] = color(j - 3);
		}
	}
	int idx = 2 * (numGrid * 2 + 1) * 12;
	g_grid_data[idx] =g_grid_data[idx + 2] = g_grid_data[idx + 6] = g_grid_data[idx + 8] = 0;
	g_grid_data[idx + 1] = -range;
	g_grid_data[idx + 7] = range;
	color = Vector3f(.6, .3, .3);
	for (int j = 3; j < 6; ++j) {
		g_grid_data[idx + j] = g_grid_data[idx + 6 + j] = color(j - 3);
	}

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * numParticle, g_Dof_buffer_data, GL_DYNAMIC_DRAW);

	GLuint colorbuffer;
	glGenBuffers(1, &colorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * numParticle, g_color_buffer_data, GL_STATIC_DRAW);

	GLuint gridbuffer;
	glGenBuffers(1, &gridbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, gridbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_grid_data), g_grid_data, GL_STATIC_DRAW);

	uint8_t pixel[height * width * 3];

	/************************* Main loop ***************************/
	while ( !sim.isQuit() && glfwWindowShouldClose(window) == 0 ) {
 		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glUseProgram(programID);

		view = camera->getLookAt();
		MVP = projection * view;
		Matrix4f f_mvp = MVP.cast<float>();
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, f_mvp.data());	// column-major order, must be float

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLdouble) * 3 * numParticle, g_Dof_buffer_data, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(
			0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,          // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		// 2nd attricute buffer:  colors
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		// Draw the hair strand !
        for (int i = 0; i < numStrand; ++i) {
			glDrawArrays(GL_LINE_STRIP, startIndex[i], startIndex[i + 1] - startIndex[i]);
        }

		glBindBuffer(GL_ARRAY_BUFFER, gridbuffer);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void*)0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));
		glDrawArrays(GL_LINES, 0, 4 * (numGrid * 2 + 1) + 2);

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);

#ifdef OUTPUT_PNG
		if (sim.shouldSave()) {
			glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)pixel);
			savePNG(pixel, sim.getStep());
			sim.shouldSave() = false;
		}
#endif
		TwDraw(); 

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();
	};

	/************************** clear up and exit *************************/
	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &colorbuffer);
	glDeleteBuffers(1, &gridbuffer);
	glDeleteProgram(programID);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW nand AntTweakBar
	glfwTerminate();

    return 0;
}

bool savePNG(const uint8_t* pixel, int step) {
	png::image<png::rgb_pixel> image(width, height);

	int idx = 0;
	for (int y = height - 1; y >= 0; --y) {		// inverse 
		for (int x = 0; x < width; ++x) {
			image[y][x] = png::rgb_pixel(pixel[idx], pixel[idx + 1], pixel[idx + 2]);
			idx += 3;
		}
	}

	std::stringstream ss;
	ss << outputDir << step << ".PNG";
	image.write(ss.str());
}