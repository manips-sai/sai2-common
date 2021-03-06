// This tests the grasp matrix function of sai2 model interface.

#include <Sai2Model.h>
#include <Sai2Graphics.h>

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/linkage.urdf";
const string robot_name = "Lkg";
const string camera_name = "camera_fixed";

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fRotPanTilt = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robot
	auto linkage = new Sai2Model::Sai2Model(robot_file, false);

	// Make perfect tetrahedron
	linkage->_q << 54.7356 /180.0*M_PI, 54.7356 /180.0*M_PI, 54.7356 /180.0*M_PI;
	linkage->updateModel();

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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "test01", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	MatrixXd G, G_inverse;
	Matrix3d R;
	Vector3d center_point = Vector3d::Zero();

	//----------------------------------------
	// test dual contact case with surface contact at one side and point at the other
	//----------------------------------------
	// vector<string> link_names;
	// link_names.push_back("link0");
	// link_names.push_back("link1");

	// vector<Vector3d> pos_in_links;
	// pos_in_links.push_back(Vector3d(1,0,0));
	// pos_in_links.push_back(Vector3d(1,0,0));

	// vector<Sai2Model::ContactNature> contact_natures;
	// contact_natures.push_back(Sai2Model::SurfaceContact);
	// contact_natures.push_back(Sai2Model::PointContact);

	linkage->addEnvironmentalContact("link0", Vector3d(1,0,0), Matrix3d::Identity(), Sai2Model::ContactType::SurfaceContact);
	linkage->addEnvironmentalContact("link1", Vector3d(1,0,0), Matrix3d::Identity(), Sai2Model::ContactType::PointContact);
	
	// for 2 contact points. the grasp matrix is given in the local frame of the virtual linkage
	// More precisely, given the external forces and moments in world frame, we get
	// the support forces and moments in local frame, as well as the internal moments along the y and z axis in local frame
	// The local frame is described by the matrix R
	// linkage->environmentalGraspMatrixAtGeometricCenter(G, R, center_point, link_names, pos_in_links, contact_natures);
	linkage->environmentalGraspMatrixAtGeometricCenter(G, G_inverse, R, center_point);
	// G.block<3,9>(0,0) = R*G.block<3,9>(0,0);
	// G.block<3,9>(3,0) = R*G.block<3,9>(3,0);

	cout << "--------------------------------------------" << endl;
	cout << "                  2 contacts                " << endl;
	cout << "--------------------------------------------" << endl;
	cout << "center point : " << center_point.transpose() << endl << endl;
	cout << "Grasp matrix : \n" << G << endl << "R : \n" << R << endl << endl;
	cout << "Grasp matrix inverse : \n" << G_inverse << endl << "R : \n" << R << endl << endl;
	cout << endl << endl;

	//----------------------------------------
	// test 3 contact case with one surface contact
	//----------------------------------------
	// link_names.clear();
	// link_names.push_back("link0");
	// link_names.push_back("link1");
	// link_names.push_back("link2");

	// pos_in_links.clear();
	// pos_in_links.push_back(Vector3d(1,0,0));
	// pos_in_links.push_back(Vector3d(1,0,0));
	// pos_in_links.push_back(Vector3d(1,0,0));

	// contact_natures.clear();
	// contact_natures.push_back(Sai2Model::SurfaceContact);
	// contact_natures.push_back(Sai2Model::PointContact);
	// contact_natures.push_back(Sai2Model::PointContact);

	linkage->addEnvironmentalContact("link2", Vector3d(1,0,0), Matrix3d::Identity(), Sai2Model::ContactType::PointContact);

	// for 3 contact points, everything is given in world frame.
	// the tensions are in the order 1-2, 1-3, 2-3.
	linkage->environmentalGraspMatrixAtGeometricCenter(G, G_inverse, R, center_point);
	
	cout << "--------------------------------------------" << endl;
	cout << "                  3 contacts                " << endl;
	cout << "--------------------------------------------" << endl;

	cout << "center point : " << center_point.transpose() << endl << endl;
	cout << "Grasp matrix : \n" << G << endl << "R : \n" << R << endl << endl;
	cout << "Grasp matrix : \n" << G_inverse << endl << "R : \n" << R << endl << endl;

	// while window is open:
    while (!glfwWindowShouldClose(window))
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, linkage);
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
	
		// move scene camera as required
    	// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
    	Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Matrix3d m_tilt; m_tilt = AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Matrix3d m_pan; m_pan = AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
	    }
	    graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);

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
	bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		default:
			break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}