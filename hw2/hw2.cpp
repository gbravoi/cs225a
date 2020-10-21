#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

//plots
#include <cmath>  // std::exp, std::cos
#include <pybind11/embed.h>  // py::scoped_interpreter
#include <pybind11/stl.h>    // bindings from C++ STL containers to Python types
namespace py = pybind11;



// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

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
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.1);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	//useful
	VectorXd q_current;
	Eigen::VectorXd g(robot->dof());


	const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof());
	std::vector< float > time_vector;
	std::vector< float > q1;
	std::vector< float > q2;
	std::vector< float > q3;
	std::vector< float > q4;
	std::vector< float > q5;
	std::vector< float > q6;
	std::vector< float > q7;
	std::vector< float > x_v;
	std::vector< float > y_v;
	std::vector< float > z_v;
	std::vector< float > x_v_d;
	std::vector< float > y_v_d;
	std::vector< float > z_v_d;
	string plot_title;
	ofstream file_f1;
	file_f1.open ("/home/question_f1.txt");
	file_f1 << "hola \n";
	file_f1.close();
	
	int controller_number = QUESTION_4;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5


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
			
		

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			
			//kp
			VectorXd kp=VectorXd::Zero(dof) ;
			kp << 400.0,400.0,400.0,400.0,400.0,400.0,50.0;
			MatrixXd Kp_m = kp.asDiagonal();
			//kv
			VectorXd kv=VectorXd::Zero(dof);
			kv << 50.0,50.0,50.0,50.0,50.0,50.0,-0.33;
			MatrixXd Kv_m = kv.asDiagonal();

		 	//qdes
			VectorXd q_desired=VectorXd::Zero(dof) ;
			q_desired<< 0, -1.0*M_PI/4.0, 0.0,(-125.0*M_PI)/180,0.0,(80.0*M_PI)/180,0.1;

			//gravity
			robot->gravityVector(g); 

			//mass matrix in joint space
			Eigen::MatrixXd A=robot->_M;

			//coriolis
			Eigen::VectorXd b(robot->dof());
			robot->coriolisForce(b);

			command_torques=(-1*Kp_m*(robot->_q-q_desired)-Kv_m*(robot->_dq))+b+g;  // change to the control torques you compute
			

			q_current=robot->_q;
			plot_title="Problem 1.e";
			q7.push_back(q_current.coeff(6));
			time_vector.push_back(time);
			
 
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 200.0;      // chose your p gain
			double kv = 25.0;      // chose your d gain
			double kvj=20; //joint damping

			//desired position
			Eigen::Vector3d ee_pos_des;
			ee_pos_des << 0.3, 0.1, 0.5;

			//current position
			Eigen::Vector3d ee_pos;
			robot->position(ee_pos, link_name, pos_in_link);
			//current speed
			//speed
			Eigen::Vector3d ee_v;
			robot->linearVelocity(ee_v,link_name, pos_in_link);

			//Robot linear jacobian in end efector
			robot->Jv(Jv, link_name, pos_in_link);
			//lambda ee
			robot->taskInertiaMatrix(Lambda, Jv);
			//gravity
			robot->gravityVector(g); 
			//nullspace
			robot->nullspaceMatrix(N, Jv);

			//force
			Eigen::VectorXd F;
			F=Lambda*(-kp*(ee_pos-ee_pos_des)-kv*ee_v);

			//command_torques=Jv.transpose()*F+g; //2a
			//command_torques=Jv.transpose()*F+g -kvj*robot->_dq; //2c
			command_torques=Jv.transpose()*F+g +N.transpose()*robot->_M*(-kvj*robot->_dq); //2d

			//save position
			q_current=robot->_q;
			q1.push_back(q_current.coeff(0));
			q2.push_back(q_current.coeff(1));
			q3.push_back(q_current.coeff(2));
			q4.push_back(q_current.coeff(3));
			q5.push_back(q_current.coeff(4));
			q6.push_back(q_current.coeff(5));
			q7.push_back(q_current.coeff(6));
			x_v.push_back(ee_pos.coeff(0));
			y_v.push_back(ee_pos.coeff(1));
			z_v.push_back(ee_pos.coeff(2));
			x_v_d.push_back(ee_pos_des.coeff(0));
			y_v_d.push_back(ee_pos_des.coeff(1));
			z_v_d.push_back(ee_pos_des.coeff(2));
			time_vector.push_back(time);
			plot_title="P2";
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{

			double kp = 200.0;      // chose your p gain
			double kv = 25.0;      // chose your d gain
			double kvj=20; //joint damping


			//desired position
			Eigen::Vector3d ee_pos_des;
			ee_pos_des << 0.3, 0.1, 0.5;

			//current position
			Eigen::Vector3d ee_pos;
			robot->position(ee_pos, link_name, pos_in_link);
			//current speed
			//speed
			Eigen::Vector3d ee_v;
			robot->linearVelocity(ee_v,link_name, pos_in_link);

			//Robot linear jacobian in end efector
			robot->Jv(Jv, link_name, pos_in_link);
			//lambda ee
			robot->taskInertiaMatrix(Lambda, Jv);
			//gravity
			robot->gravityVector(g); 
			//nullspace
			robot->nullspaceMatrix(N, Jv);
			//gravity vector in task space p
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			Eigen::VectorXd p;
			p=J_bar.transpose()*g;

			//force
			Eigen::VectorXd F;
			F=Lambda*(-kp*(ee_pos-ee_pos_des)-kv*ee_v)+p;

			command_torques=Jv.transpose()*F +N.transpose()*robot->_M*(-kvj*robot->_dq); 

			//save position
			q_current=robot->_q;
			q1.push_back(q_current.coeff(0));
			q2.push_back(q_current.coeff(1));
			q3.push_back(q_current.coeff(2));
			q4.push_back(q_current.coeff(3));
			q5.push_back(q_current.coeff(4));
			q6.push_back(q_current.coeff(5));
			q7.push_back(q_current.coeff(6));
			x_v.push_back(ee_pos.coeff(0));
			y_v.push_back(ee_pos.coeff(1));
			z_v.push_back(ee_pos.coeff(2));
			x_v_d.push_back(ee_pos_des.coeff(0));
			y_v_d.push_back(ee_pos_des.coeff(1));
			z_v_d.push_back(ee_pos_des.coeff(2));
			time_vector.push_back(time);
			plot_title="P3";
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			double kp = 200.0;      // chose your p gain
			double kv = 25.0;      // chose your d gain
			double kvj=20; //joint damping
			double kpj=20; //joint p gain

			//desired position
			Eigen::Vector3d ee_pos_des;
			ee_pos_des << 0.3+0.1*sin(M_PI*time), 0.1+0.1*cos(M_PI*time), 0.5;

			//desired velocity
			Eigen::Vector3d ee_v_des;
			ee_v_des << 0.1*M_PI*cos(M_PI*time), -0.1*M_PI*sin(M_PI*time), 0.0;

			//desired joint position
			VectorXd q_des=VectorXd::Zero(dof);
			

			//current position
			Eigen::Vector3d ee_pos;
			robot->position(ee_pos, link_name, pos_in_link);
			//current speed
			//speed
			Eigen::Vector3d ee_v;
			robot->linearVelocity(ee_v,link_name, pos_in_link);

			//Robot linear jacobian in end efector
			robot->Jv(Jv, link_name, pos_in_link);
			//lambda ee
			robot->taskInertiaMatrix(Lambda, Jv);
			//gravity
			robot->gravityVector(g); 
			//nullspace
			robot->nullspaceMatrix(N, Jv);
			//gravity vector in task space p
			//robot->dynConsistentInverseJacobian(J_bar, Jv);
			J_bar = robot->_M_inv * Jv.transpose() * Lambda;
			Eigen::VectorXd p;
			p=J_bar.transpose()*g;

			//force
			Eigen::VectorXd F;
			F=Lambda*(-kp*(ee_pos-ee_pos_des)-kv*(ee_v-ee_v_des))+p;

			//command_torques=Jv.transpose()*F +N.transpose()*robot->_M*(-kvj*robot->_dq); //i
			//command_torques=Jv.transpose()*((-kp*(ee_pos-ee_pos_des)-kv*(ee_v-ee_v_des))+p) +N.transpose()*robot->_M*(-kvj*robot->_dq); //ii
			command_torques=Jv.transpose()*F +N.transpose()*robot->_M*(-kpj*(robot->_dq-q_des)); //iii
			//command_torques=Jv.transpose()*F +N.transpose()*(robot->_M*(-kpj*(robot->_dq-q_des))+g); //iv

			//save position
			q_current=robot->_q;
			q1.push_back(q_current.coeff(0));
			q2.push_back(q_current.coeff(1));
			q3.push_back(q_current.coeff(2));
			q4.push_back(q_current.coeff(3));
			q5.push_back(q_current.coeff(4));
			q6.push_back(q_current.coeff(5));
			q7.push_back(q_current.coeff(6));
			x_v.push_back(ee_pos.coeff(0));
			y_v.push_back(ee_pos.coeff(1));
			z_v.push_back(ee_pos.coeff(2));
			x_v_d.push_back(ee_pos_des.coeff(0));
			y_v_d.push_back(ee_pos_des.coeff(1));
			z_v_d.push_back(ee_pos_des.coeff(2));
			time_vector.push_back(time);
			plot_title="P4";
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
		
		//VectorXd q_current=robot->_q;
		//cout <<q_current.coeff(0)<< " " << q_current.coeff(1) << " " <<q_current.coeff(2)<<" " <<q_current.coeff(3)<<" " <<q_current.coeff(4) <<" " <<q_current.coeff(5) <<" " <<q_current.coeff(6) <<" " <<q_current.coeff(7)  << endl;


	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	//create plot
	if(controller_number == QUESTION_1){
		// Start the Python interpreter
		py::scoped_interpreter guard{};
		using namespace py::literals;

		// Save the necessary local variables in a Python dict
		py::dict locals = py::dict{
			"q7"_a = q7,
			"time"_a = time_vector,
			"plot_title"_a=plot_title,
		};

		// Execute Python code, using the variables saved in `locals`
		py::exec(R"(
		
		import matplotlib.pyplot as plt

		plt.figure()
		plt.suptitle(plot_title)
		plt.plot(time,q7,'r',label='q7')
		plt.xlabel('Time [s]')
		plt.ylabel('Angle [rad]')
		plt.legend()
		plt.show()
		
		)",
				py::globals(), locals);
	}else if(controller_number == QUESTION_2 or controller_number == QUESTION_3 or controller_number == QUESTION_4){
				// Start the Python interpreter
		py::scoped_interpreter guard{};
		using namespace py::literals;

		// Save the necessary local variables in a Python dict
		py::dict locals = py::dict{
			"q1"_a = q1,
			"q2"_a = q2,
			"q3"_a = q3,
			"q4"_a = q4,
			"q5"_a = q5,
			"q6"_a = q6,
			"q7"_a = q7,
			"x_v"_a = x_v,
			"y_v"_a = y_v,
			"z_v"_a = z_v,
			"x_v_d"_a = x_v_d,
			"y_v_d"_a = y_v_d,
			"z_v_d"_a = z_v_d,
			"time"_a = time_vector,
			"plot_title"_a=plot_title,
		};

		// Execute Python code, using the variables saved in `locals`
		py::exec(R"(
		
		import matplotlib.pyplot as plt

		plt.figure()
		plt.suptitle(plot_title+" End effector position")
		plt.plot(time,x_v,'r',label='x')
		plt.plot(time,y_v,'b',label='y')
		plt.plot(time,z_v,'g',label='z')
		plt.plot(time,x_v_d,color='r',linestyle='--',label='x desired')
		plt.plot(time,y_v_d,color='b',linestyle='--',label='y desired')
		plt.plot(time,z_v_d,color='g',linestyle='--',label='z desired')
		plt.xlabel('Time [s]')
		plt.ylabel('Position [m]')
		plt.legend()
		plt.show()

		plt.figure()
		plt.suptitle(plot_title+" Joints")
		plt.plot(time,q1,label='q1')
		plt.plot(time,q2,label='q2')
		plt.plot(time,q3,label='q3')
		plt.plot(time,q4,label='q4')
		plt.plot(time,q5,label='q5')
		plt.plot(time,q6,label='q6')
		plt.plot(time,q7,label='q7')
		plt.xlabel('Time [s]')
		plt.ylabel('Angle [rad]')
		plt.legend()
		plt.show()
		
		)",
				py::globals(), locals);
	}


	return 0;
}
