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
#include "uiforce/UIForceWidget.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/rpbot.urdf";
const string robot_name1 = "RPBot";
const string camera_name = "camera_fixed";

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow *window, int button, int action, int mods);

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);

// flags for ui widget click
bool fRobotLinkSelect = false;
Eigen::Vector3d ui_force;
Eigen::VectorXd ui_force_command_torques;


int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);

	// load robots
	auto robot1 = new Sai2Model::Sai2Model(robot_file, false);
	sim->getJointPositions(robot_name1, robot1->_q);
	robot1->updateModel();

	// init click force widget 
	auto ui_force_widget = new UIForceWidget(robot_name1, robot1, graphics);
	ui_force_widget->setEnable(false);

	ui_force_widget->_spring_k = 50.0;
	ui_force_widget->_max_force = 100.0;

	int dof = robot1->dof();
	ui_force.setZero();
	ui_force_command_torques.setZero(dof);

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
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation
	thread sim_thread(simulation, robot1, sim, ui_force_widget);

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

	    // detect click to the link
		ui_force_widget->setEnable(fRobotLinkSelect);
		if (fRobotLinkSelect)
		{
			double cursorx, cursory;
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;

			// get current cursor position
			glfwGetCursorPos(window, &cursorx, &cursory);

			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

			int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
			int viewy = floor(cursory / wheight_scr * wheight_pix);

			ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
			//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
			// then drag the mouse over a link to start applying a force to it.
		}

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

	double kp_p = 50.0;
	double kv_p = 14.0;
	Eigen::VectorXd tau(2);
	tau.setZero();

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

		// compute torques with hold position controller
		tau = (-kp_p * robot->_q - kv_p * robot->_dq);

		// set torques
		sim->setJointTorques(robot_name1, tau + ui_force_command_torques);
	}
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {
	fSimulationRunning = true;
	int dof = robot->dof();

	// create a timer
	LoopTimer timer; timer.initializeTimer();
	timer.setLoopFrequency(1000); //1kHz timer
	double last_time = timer.elapsedTime(); //secs
	while (fSimulationRunning) {
		timer.waitForNextLoop();
		// integrate forward
		double curr_time = timer.elapsedTime();
		sim->integrate(curr_time - last_time);

		// get ui force and torques
		if(fRobotLinkSelect)
		{
			ui_force_widget->getUIForce(ui_force);
			ui_force_widget->getUIJointTorques(ui_force_command_torques);
		}
		else
		{
			ui_force.setZero();
			ui_force_command_torques.setZero(dof);
		}

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

void mouseClick(GLFWwindow *window, int button, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button)
	{
	// if right click: apply force to robot
	case GLFW_MOUSE_BUTTON_RIGHT:
		fRobotLinkSelect = set;
		break;
	default:
		break;
	}
}