// This example demonstrates the use of a spherical joint

#include <Sai2Model.h>
#include <Sai2Simulation.h>
#include <chai3d/CDynamicWorld.h>
#include <Sai2Graphics.h>
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <thread>
#include <cmath>

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/spbot.urdf";
const string robot_name1 = "SPBot";
const string camera_name = "camera_fixed";

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);

	// load robots
	auto robot1 = new Sai2Model::Sai2Model(robot_file, false);
	sim->getJointPositions(robot_name1, robot1->_q);
	robot1->updateModel();

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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "07-spherical-joint", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot1, sim);

	// next start the control thread
	thread ctrl_thread(control, robot1, sim);

    // while window is open:
    while (!glfwWindowShouldClose(window))
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name1, robot1);
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
	}
	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	// control variables

	double kp_p = 250.0;
	double kv_p = 4.0;
	Eigen::MatrixXd Jw(3,4);
	Eigen::VectorXd tau(4);
	tau.setZero();
	Eigen::VectorXd G(4);

	// create a timer
	LoopTimer timer; timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
	while (fSimulationRunning) {
		timer.waitForNextLoop();

		// update model models
		sim->getJointPositions(robot_name1, robot->_q);
		sim->getJointVelocities(robot_name1, robot->_dq);
		// cout << robot->_q.transpose() << endl;
		// cout << "\t\t"<< robot->_dq.transpose() << endl;
		robot->updateModel();

		// get angular velocity jacobian and set moment in world frame
		robot->Jw(Jw, "link0");
		robot->gravityVector(G);

		// control torques
		tau = robot->_M*((Jw.transpose())*Eigen::Vector4d(0.0, 6.0*sin(0.6*timer.elapsedTime()), 1.0*sin(timer.elapsedTime()), 0.0));
		if (isnan(tau[0])) {
			cout << "q" << endl;
			cout << robot->_q.transpose() << endl;
			cout << "Jw" << endl;
			cout << Jw << endl;
			cout << "A" << endl;
			cout << robot->_M << endl;
			cout << "tau" << endl;
			cout << tau.transpose() << endl;
			exit(1);
		}
		tau[3] = -kp_p*(robot->_q[3]) - kv_p*robot->_dq[3] + G[3];

		// set torques
		sim->setJointTorques(robot_name1, tau);
	}
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer; timer.initializeTimer();
	timer.setLoopFrequency(1000); //1kHz timer
	double last_time = timer.elapsedTime(); //secs
	while (fSimulationRunning) {
		timer.waitForNextLoop();
		// integrate forward
		double curr_time = timer.elapsedTime();
		sim->integrate(curr_time - last_time);
		// update last time
		last_time = curr_time;
	}
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

