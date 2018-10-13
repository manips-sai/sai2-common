// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Sai2Simulation virtual world. A graphics model of it is also shown using 
// Chai3D.

#include <Sai2Model.h>
#include <Sai2Simulation.h>
#include <Sai2Graphics.h>
#include "timer/LoopTimer.h"
#include "trajectory_generation/OTG.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

#include <thread>
#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/rrrbot.urdf";
const string robot_name = "RRRBot";
const string camera_name = "camera_fixed";

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);

	// offset a joint initial condition
	sim->getJointPositions(robot_name, robot->_q);
	robot->_q << 0.5, -0.2, 0.3;
	robot->updateModel();
	sim->setJointPositions(robot_name, robot->_q);
	
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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "07-trajectory_generation", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

    // while window is open:
    while (!glfwWindowShouldClose(window))
	{


		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
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

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim)
{
	// simulation frequency
	double sim_frequency = 2000;

	// create trajectory generation object
	OTG* trajectory_generator = new OTG(robot->dof(), 1/sim_frequency);
	trajectory_generator->setMaxVelocity(0.2);
	trajectory_generator->setMaxAcceleration(0.1);
	trajectory_generator->updateState(robot->_q, robot->_dq);

	Eigen::VectorXd goal_position = Eigen::VectorXd::Zero(robot->dof());
	goal_position << -1.2, 0.9, -0.7;
	trajectory_generator->setGoalPosition(goal_position);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_frequency); 
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs
	double last_time = start_time;

	unsigned long long sim_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// set new robot position and velocity
		trajectory_generator->updateState(robot->_q, robot->_dq);
		trajectory_generator->computeNextState(robot->_q, robot->_dq);
		sim->setJointPositions(robot_name, robot->_q);
		sim->setJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		if(sim_counter % 1000 == 0)
		{
			cout << "robot joint positions : " << robot->_q.transpose() << endl;
			cout << "robot joint velocities : " << robot->_dq.transpose() << endl;
			cout << endl;
		}

		sim_counter++;
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
