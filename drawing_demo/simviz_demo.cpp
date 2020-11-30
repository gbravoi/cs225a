#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <signal.h>
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include "uiforce/UIForceWidget.h"  // used for right-click drag interaction in window 
#include "force_sensor/ForceSensorSim.h"  // references src folder in sai2-common directory 
#include "force_sensor/ForceSensorDisplay.h"

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

// specify urdf and robots 
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/panda_collision.urdf";
const string robot_name = "panda_collision";
const string camera_name = "camera_fixed";
const string ee_link_name = "link7";

RedisClient redis_client;

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::panda::sensors::dq";
const std::string PAINT_LOCATION_KEY = "cs225a::canvas::coordinate";
const std::string PAINT_FORCE_KEY = "cs225a::canvas::force";
const std::string PAINT_COLOR_KEY = "cs225a::canvas::color";
const std::string CANVAS_RESET_KEY = "cs225a::canvas::reset"; // "1" = reset; "0" is default

// - read:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs225a::robot::panda::actuators::fgc";

// force sensor
ForceSensorSim* force_sensor;

// display widget for forces at end effector
ForceSensorDisplay* force_display;

// simulation thread
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, chai3d::cMesh* canvas, \
	chai3d::cImagePtr canvasOriginal, UIForceWidget *ui_force_widget);

// callback for canvas update 
void updateCanvas(chai3d::cMesh* canvas, chai3d::cImagePtr canvasOriginal);

// callback for canvas reset
void resetCanvas(chai3d::cMesh* canvas);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->showLinkFrame(true, robot_name, ee_link_name, 0.15);  // can add frames for different links
	// graphics->showLinkFrame(true, robot_name, "link0", 0.15);  // can add frames for different links

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q(0) = -0.8;
	robot->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setJointPositions(robot_name, robot->_q);

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(0.5);
    sim->setCoeffFrictionDynamic(0.5);

	// initialize force sensor: needs Sai2Simulation sim interface type
	force_sensor = new ForceSensorSim(robot_name, ee_link_name, Eigen::Affine3d::Identity(), robot);
	force_display = new ForceSensorDisplay(force_sensor, graphics);

	// init redis client values 
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq); 

	VectorXd color_(3); 
	color_ << 0, 0, 0;  // R, G, B (range between 0 and 255)
	VectorXd location_(2);
	location_ << -100, -100;  // total size of canvas based on createPlane (set off the canvas initially)
	VectorXd force_(1);  // set force for lighter/darker and smaller/larger strokes 
	force_ << 1;
	redis_client.setEigenMatrixJSON(PAINT_LOCATION_KEY, location_);
	redis_client.setEigenMatrixJSON(PAINT_COLOR_KEY, color_);
	redis_client.setEigenMatrixJSON(PAINT_FORCE_KEY, force_);
	redis_client.set(CANVAS_RESET_KEY, "0");

    // create canvas
    chai3d::cMesh* canvas = new chai3d::cMesh();

    // create a plane
    chai3d::cCreatePlane(canvas, 1.0, 1.0);

    // add object to world
    graphics->_world->addChild(canvas);

    // set the position of the object
    canvas->setLocalPos(-0.5, 0.0, 1.0);  // relative to the world origin, specifying the CENTER of the plane wrt the world origin 
    canvas->rotateAboutGlobalAxisRad(chai3d::cVector3d(0,1,0), chai3d::cDegToRad(90));
    canvas->rotateAboutGlobalAxisRad(chai3d::cVector3d(1,0,0), chai3d::cDegToRad(90));

    // set graphic properties
    canvas->m_texture = chai3d::cTexture2d::create();
    bool fileload = canvas->m_texture->loadFromFile("./resources/canvas.jpg");

    // create a copy of canvas so that we can clear page when requested
	chai3d::cImagePtr canvasOriginal = canvas->m_texture->m_image->copy();

    // we disable lighting properties for canvas
    canvas->setUseMaterial(false);

    // enable texture mapping
    canvas->setUseTexture(true);

    // create normal map from texture data
    chai3d::cNormalMapPtr normalMap = chai3d::cNormalMap::create();
    normalMap->createMap(canvas->m_texture);
    canvas->m_normalMap = normalMap;

	// set haptic properties
    canvas->m_material->setStaticFriction(0.50);
    canvas->m_material->setDynamicFriction(0.50);
    canvas->m_material->setTextureLevel(1.5);  // intensity of texture

    // output pixel width and height
    int canvas_width = canvas->m_texture->m_image->getWidth();
    int canvas_height = canvas->m_texture->m_image->getHeight();
    cout << canvas_width << ", " << canvas_height << endl;  // (1024, 1024) pixel 

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
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "cs225a - drawing-demo", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// init click force widget 
	auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
	ui_force_widget->setEnable(false);

	// cache variables
	double last_cursorx, last_cursory;

	// initialize glew
	glewInitialize();

	// start simulation thread
	thread sim_thread(simulation, robot, sim, canvas, canvasOriginal, ui_force_widget);

	// start canvas thread
	// thread canvas_thread(updateCanvas, canvas);

	// while window is open:
	// while (!glfwWindowShouldClose(window) && fSimulationRunning)
	while (!glfwWindowShouldClose(window))
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		force_display->update();
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
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
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
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

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

			if (cursorx > 0 && cursory > 0)
			{
				ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
				//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
				// then drag the mouse over a link to start applying a force to it.
			}
		}

	}

	// wait for simulation to finish
	fSimulationRunning = false;
	sim_thread.join();
	// canvas_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void resetCanvas(chai3d::cMesh* canvas, chai3d::cImagePtr canvasOriginal)
{
	// move drawing location away from current position
	redis_client.set(PAINT_LOCATION_KEY, "[-100, -100]");

    // copy original image of canvas to texture
    canvasOriginal->copyTo(canvas->m_texture->m_image);

    // update texture
    canvas->m_texture->markForUpdate();
}

//------------------------------------------------------------------------------
void updateCanvas(chai3d::cMesh* canvas)
{
	// init containers
	Eigen::VectorXd location_, color_, force_;

	// get brush location, force, and color specification from redis-server
	location_ = redis_client.getEigenMatrixJSON(PAINT_LOCATION_KEY);
	force_ = redis_client.getEigenMatrixJSON(PAINT_FORCE_KEY);
	color_ = redis_client.getEigenMatrixJSON(PAINT_COLOR_KEY);	

	int px = location_(0);
	int py = location_(1);
	double r_color = color_(0);
	double g_color = color_(1);
	double b_color = color_(2);

	// paint color at tool position
	double K_INK = 30;
	double K_SIZE = 10;
	int BRUSH_SIZE = 25;
	double force = force_(0); 
	double timeInterval = 0.001;

	double size = chai3d::cClamp((K_SIZE * force), 1.0, (double)(BRUSH_SIZE));
	for (int x=-BRUSH_SIZE; x<BRUSH_SIZE; x++)
	{
	    for (int y=-BRUSH_SIZE; y<BRUSH_SIZE; y++)
	    {                        
	        // compute new color percentage
	        double distance = sqrt((double)(x*x+y*y));
	        if (distance <= size)
	        {
	            // get color at location
	            chai3d::cColorb newColor, color;
	            canvas->m_texture->m_image->getPixelColor(px+x, py+y, color);
	            
	            // compute color factor based of pixel position and force interaction 
	            double factor = chai3d::cClamp(K_INK * timeInterval * chai3d::cClamp(force, 0.0, 10.0) * chai3d::cClamp(1 - distance/size, 0.0, 1.0), 0.0, 1.0);

	            // compute new color
	            GLubyte R = (GLubyte)chai3d::cClamp(((1.0 - factor) * (double)color.getR() + factor * (double)r_color), 0.0, 255.0);
	            GLubyte G = (GLubyte)chai3d::cClamp(((1.0 - factor) * (double)color.getG() + factor * (double)g_color), 0.0, 255.0);
	            GLubyte B = (GLubyte)chai3d::cClamp(((1.0 - factor) * (double)color.getB() + factor * (double)b_color), 0.0, 255.0);  
	            newColor.set(R, G, B);

	            // assign new color to pixel
	            int pixelX = px + x;
	            int pixelY = py + y;
	            if ((pixelX >= 0) && (pixelY >= 0) && (pixelX < canvas->m_texture->m_image->getWidth()) && (pixelY < canvas->m_texture->m_image->getHeight()))
	            {
	                canvas->m_texture->m_image->setPixelColor(pixelX, pixelY, newColor);
	            }
	        }
	    }
	}	

	// mark for update
	canvas->m_texture->markForUpdate(); 
}


//------------------------------------------------------------------------------

void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, chai3d::cMesh* canvas, chai3d::cImagePtr canvasOriginal, UIForceWidget *ui_force_widget)
{
	// prepare simulation
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double time_slowdown_factor = 1.0;  // adjust to higher value (i.e. 2) to slow down simulation by this factor relative to real time (for slower machines)
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime() / time_slowdown_factor; // secs
	double last_time = start_time;

	// init control variables 
	VectorXd g(dof);
	Eigen::Vector3d ui_force;
	ui_force.setZero();
	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();
	double kvj = 10;

	// setup redis client data container for pipeset (batch write)
	std::vector<std::pair<std::string, std::string>> redis_data(10);  // set with the number of keys to write 

	fSimulationRunning = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// get gravity torques
		robot->gravityVector(g);

		// read arm torques from redis and apply to simulated robot
		command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);

		// get forces from interactive screen 
		ui_force_widget->getUIForce(ui_force);
		ui_force_widget->getUIJointTorques(ui_force_command_torques);

		if (fRobotLinkSelect)
			sim->setJointTorques(robot_name, command_torques + ui_force_command_torques - robot->_M*kvj*robot->_dq + g);
		else
			sim->setJointTorques(robot_name, command_torques - robot->_M*kvj*robot->_dq + g);  // can comment out the joint damping if controller does this 

		// integrate forward
		double curr_time = timer.elapsedTime() / time_slowdown_factor;
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// update canvas
		updateCanvas(canvas);

		// check if need to reset canvas (might be slow depending on sim integrate)
		if (redis_client.get(CANVAS_RESET_KEY) == "1") {
			resetCanvas(canvas, canvasOriginal);
			redis_client.set(CANVAS_RESET_KEY, "0");
		}

		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime() / time_slowdown_factor;
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}


//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

bool limitCheck(double a, double b) {
    if (a > 0 && b > 0) {
        return true;
    }
    else if (a < 0 && b < 0) {
        return true;
    }
    else {
        return false;
    }
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window, GL_TRUE);
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
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
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
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

