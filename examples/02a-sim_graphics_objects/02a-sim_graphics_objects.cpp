// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Sai2Simulation virtual world. A graphics model of it is also shown using 
// Chai3D.

// #include <Sai2Model.h>
#include <Sai2Simulation.h>
#include <Sai2Graphics.h>
#include <dynamics3d.h>

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

using namespace std;
using namespace chai3d;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/dummy.urdf";
const string camera_name = "camera_fixed";
const string object_name = "floating_box";

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);

	// TODO : there is a linker error if no robot is created
	auto dummy_robot = new Sai2Model::Sai2Model(robot_file, false);

	/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "02-sim_graphics_urdfmodel", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// loop counter
	unsigned long long counter;

    // while window is open:
    while (!glfwWindowShouldClose(window))
	{
		// update simulation by 1ms
		sim->integrate(0.01);

		// get object pose
		Eigen::Quaterniond object_ori;
		Eigen::Vector3d object_pos, object_lin_vel, object_ang_vel;
		sim->getObjectPosition(object_name, object_pos, object_ori);
		sim->getObjectVelocity(object_name, object_lin_vel, object_ang_vel);

		if(counter % 10 == 0)
		{
			cout << "object position : " << object_pos.transpose() << endl;
			cout << "object orientation\n" << object_ori.toRotationMatrix() << endl;
			cout << "object linear velocity : " << object_lin_vel.transpose() << endl;
			cout << "object angular velocity : " << object_ang_vel.transpose() << endl;
			cout << endl;
		}

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateObjectGraphics(object_name, object_pos, object_ori);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

	    // poll for events
	    glfwPollEvents();

	    counter++;
	}

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}
