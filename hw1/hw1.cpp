#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <math.h>

//plots
#include <cmath>  // std::exp, std::cos
#include <pybind11/embed.h>  // py::scoped_interpreter
#include <pybind11/stl.h>    // bindings from C++ STL containers to Python types

 

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4
#define QUESTION_5   5

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm_controller.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

namespace py = pybind11;


int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	
	//extra variables
	int counter=0;
	VectorXd q_previous=robot->_q;
	VectorXd q_current;
	VectorXd q_desired=VectorXd::Zero(dof) ;  // change to the desired robot joint angles for the question


	//array to save joint
	std::vector< float > time_vector;
	std::vector< float > q1;
	std::vector< float > q2;
	std::vector< float > q3;
	std::vector< float > q4;
	std::vector< float > q5;
	std::vector< float > q6;
	std::vector< float > q1d;
	std::vector< float > q2d;
	std::vector< float > q3d;
	std::vector< float > q4d;
	std::vector< float > q5d;
	std::vector< float > q6d;
	string plot_title;


	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_4;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5
		q_desired<< M_PI/2.0, -1.0*M_PI/4.0, 0.0,(-125.0*M_PI)/180,(80.0*M_PI)/180,0.0;

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 50.0;      // chose your d gain

			
			command_torques=-1*kp*(robot->_q-q_desired)-kv*(robot->_dq);  // change to the control torques you compute
			plot_title="Problem 1";
			
				
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{

			double kp = 400.0;      // chose your p gain
			double kv = 50.0;      // chose your d gain
			//gravity
			Eigen::VectorXd g(robot->dof());
			robot->gravityVector(g); 
			command_torques=-1*kp*(robot->_q-q_desired)-kv*(robot->_dq)+g;  // change to the control torques you compute
			plot_title="Problem 2";
		
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{

			double kp = 400.0;      // chose your p gain
			double kv = 50.0;      // chose your d gain
			//gravity
			Eigen::VectorXd g(robot->dof());
			robot->gravityVector(g); 

			//mass matrix in joint space
			Eigen::MatrixXd A=robot->_M;
			plot_title="Problem 3";


			command_torques=A*(-1*kp*(robot->_q-q_desired)-kv*(robot->_dq))+g;  // change to the control torques you compute
	
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{

			
			double kp = 400.0;      // chose your p gain
			double kv = 50.0;      // chose your d gain
			//gravity
			Eigen::VectorXd g(robot->dof());
			robot->gravityVector(g); 

			//mass matrix in joint space
			Eigen::MatrixXd A=robot->_M;

			//coriolis
			Eigen::VectorXd b(robot->dof());
			robot->coriolisForce(b);

			command_torques=A*(-1*kp*(robot->_q-q_desired)-kv*(robot->_dq))+b+g;  // change to the control torques you compute
			plot_title="Problem 4";

		}

		// ---------------------------  question 5 ---------------------------------------
		if(controller_number == QUESTION_5)
		{

			double kp = 400.0;      // chose your p gain
			double kv = 50.0;      // chose your d gain
			//gravity
			Eigen::VectorXd g(robot->dof());
			robot->gravityVector(g); 

			//mass matrix in joint space
			Eigen::MatrixXd A=robot->_M;

			//coriolis
			Eigen::VectorXd b(robot->dof());
			robot->coriolisForce(b);

			command_torques=A*(-1*kp*(robot->_q-q_desired)-kv*(robot->_dq))+b+g;  // change to the control torques you compute
			plot_title="Problem 5";

		}

		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************
		q_current=robot->_q;
		//save joints information
		if((q_current-q_previous).squaredNorm()>0.0017)
		{
			q_previous=q_current;
			q1.push_back(q_current.coeff(0));
			//q2.push_back(q_current.coeff(1));
			q3.push_back(q_current.coeff(2));
			q4.push_back(q_current.coeff(3));
			//q5.push_back(q_current.coeff(4));
			//q6.push_back(q_current.coeff(5));
			q1d.push_back(q_desired.coeff(0));
			q3d.push_back(q_desired.coeff(2));
			q4d.push_back(q_desired.coeff(3));
			//q3d.push_back(q_desired.coeff(0));
			//q4d.push_back(q_desired.coeff(0));
			time_vector.push_back(timer.elapsedTime());
		}else if(counter<1000){
			counter++;
			q1.push_back(q_current.coeff(0));
			//q2.push_back(q_current.coeff(1));
			q3.push_back(q_current.coeff(2));
			q4.push_back(q_current.coeff(3));
			//q5.push_back(q_current.coeff(4));
			//q6.push_back(q_current.coeff(5));
			q1d.push_back(q_desired.coeff(0));
			q3d.push_back(q_desired.coeff(2));
			q4d.push_back(q_desired.coeff(3));
			time_vector.push_back(timer.elapsedTime());
		}else{
			//break;
		} 

		// send to redis
		

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	// **********************
	// pritn array with results
	// // 
	//cout << "Out of the while loop " << endl;

	//create plot
	// Start the Python interpreter
    py::scoped_interpreter guard{};
    using namespace py::literals;

    // Save the necessary local variables in a Python dict
    py::dict locals = py::dict{
        "q1"_a = q1,
		"q3"_a = q3,
		"q4"_a = q4,
		"q1d"_a = q1d,
		"q3d"_a = q3d,
		"q4d"_a = q4d,
		"time"_a = time_vector,
		"plot_title"_a=plot_title,
    };

    // Execute Python code, using the variables saved in `locals`
    py::exec(R"(
    
    import matplotlib.pyplot as plt

    fig, (ax1, ax3,ax4) = plt.subplots(1, 3,figsize=(17,7))
    fig.suptitle(plot_title)

    ax1.set_title('Joint 1')
    ax1.plot(time,q1,'r',label='q1')
    ax1.plot(time,q1d,color='r',linestyle='--',label='q1 desired')

    ax3.set_title('Joint 3')
    ax3.plot(time,q3,'b',label='q3')
    ax3.plot(time,q3d,color='b',linestyle='--',label='q3 desired')
	
    ax4.set_title('Joint 4')
    ax4.plot(time,q4,'g',label='q4')
    ax4.plot(time,q4d,color='g',linestyle='--',label='q4 desired')
    	
    
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Angle [rad]')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Angle [rad]')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Angle [rad]')
	
    ax1.legend()
    ax3.legend()
    ax4.legend()

    plt.show()

    plt.figure()
    plt.suptitle(plot_title)
    plt.plot(time,q1,'r',label='q1')
    plt.plot(time,q3,'b',label='q3')
    plt.plot(time,q4,'g',label='q4')
    plt.plot(time,q1d,color='r',linestyle='--',label='q1 desired')
    plt.plot(time,q3d,color='b',linestyle='--',label='q3 desired')	
    plt.plot(time,q4d,color='g',linestyle='--',label='q4 desired')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [rad]')
    plt.legend()
    plt.show()
    
    )",
             py::globals(), locals);


	

	return 0;
}
