#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

//plots
#include <cmath>  // std::exp, std::cos
#include <pybind11/embed.h>  // py::scoped_interpreter
#include <pybind11/stl.h>    // bindings from C++ STL containers to Python types
namespace py = pybind11;


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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
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

	redis_client.set(CONTROLLER_RUNING_KEY, "1");

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
	std::vector< float > q4_up;
	std::vector< float > q4_down;
	std::vector< float > q6_up;
	std::vector< float > q6_down;
	std::vector< float > x_v;
	std::vector< float > y_v;
	std::vector< float > z_v;
	std::vector< float > x_v_d;
	std::vector< float > y_v_d;
	std::vector< float > z_v_d;
	std::vector< float > dphi_x;
	std::vector< float > dphi_y;
	std::vector< float > dphi_z;
	std::vector< float > v_current;
	std::vector< float > v_max_;
	string plot_title;

	int controller_number = QUESTION_4;  

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

			double kp = 100.0;      // chose your p gain
			double kv = 20.0;      // chose your d gain
			double kvj=14.0; //joint damping
			double kpj=50.0; //joint p gain

			//desired position
			Eigen::Vector3d ee_pos_des;
			ee_pos_des << 0.3+0.1*sin(M_PI*time), 0.1+0.1*cos(M_PI*time), 0.5;

			//desired velocity
			Eigen::Vector3d ee_v_des;
			ee_v_des << 0.1*M_PI*cos(M_PI*time), -0.1*M_PI*sin(M_PI*time), 0.0;
			
			//desired acceleration
			Eigen::Vector3d ee_a_des;
			ee_a_des << -0.1*pow(M_PI,2)*sin(M_PI*time), -0.1*pow(M_PI,2)*cos(M_PI*time), 0.0;

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

			//force
			Eigen::VectorXd Fa;
			Fa=Lambda*(-kp*(ee_pos-ee_pos_des)-kv*(ee_v));
			Eigen::VectorXd Fc;
			Fc=Lambda*(ee_a_des-kp*(ee_pos-ee_pos_des)-kv*(ee_v-ee_v_des));

			//command_torques=Jv.transpose()*Fa +N.transpose()*(-kpj*(robot->_q-q_des)-kvj*robot->_dq)+g; //a
			command_torques=Jv.transpose()*Fc +N.transpose()*(-kpj*(robot->_q-q_des)-kvj*robot->_dq)+g; //c

			//save plot info
			x_v.push_back(ee_pos.coeff(0));
			y_v.push_back(ee_pos.coeff(1));
			z_v.push_back(ee_pos.coeff(2));
			x_v_d.push_back(ee_pos_des.coeff(0));
			y_v_d.push_back(ee_pos_des.coeff(1));
			z_v_d.push_back(ee_pos_des.coeff(2));
			time_vector.push_back(time);
			plot_title="P1";
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{

			double kp = 100.0;      // chose your p gain
			double kv = 20.0;      // chose your d gain
			double kvj=14.0; //joint damping
			double kpj=50.0; //joint p gain

			string letter;
			letter="g";


			//desired position
			Eigen::Vector3d ee_pos_des;
			if(letter=="f" || letter=="g"){
				ee_pos_des << -0.65, -0.45,0.7;
			}else{
				ee_pos_des << -0.1, 0.15, 0.2;
			}


			

			//desired joint position
			VectorXd q_under(robot->dof());
			q_under<< (-165*M_PI)/180,(-100*M_PI)/180, (-165*M_PI)/180,(-170*M_PI)/180,(-165*M_PI)/180,0.0, (-165*M_PI)/180;  
			VectorXd q_bar(robot->dof());
			q_bar<< (165*M_PI)/180,(100*M_PI)/180, (165*M_PI)/180,(-30*M_PI)/180,(165*M_PI)/180,(210*M_PI)/180, (165*M_PI)/180; 
			VectorXd q_des(robot->dof());
			q_des=(q_under+q_bar)*0.5;
			

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
			F=Lambda*(-kp*(ee_pos-ee_pos_des)-kv*(ee_v));

			if(letter=="d"){
				command_torques=Jv.transpose()*F +N.transpose()*(-kvj*robot->_dq)+g; //d. where k_pj=2k_mid and k_vj=k_damp
			}else if(letter=="e" || letter=="f"){
				command_torques=Jv.transpose()*F +N.transpose()*(-kpj*(robot->_q-q_des))+N.transpose()*(-kvj*robot->_dq)+g; //e, f
			}else if(letter=="g"){
				command_torques=Jv.transpose()*F +(-kpj*(robot->_q-q_des))+N.transpose()*(-kvj*robot->_dq)+g; //g
			}else{
				command_torques.setZero();
			}
			

			//save plot info
			x_v.push_back(ee_pos.coeff(0));
			y_v.push_back(ee_pos.coeff(1));
			z_v.push_back(ee_pos.coeff(2));
			x_v_d.push_back(ee_pos_des.coeff(0));
			y_v_d.push_back(ee_pos_des.coeff(1));
			z_v_d.push_back(ee_pos_des.coeff(2));
			q_current=robot->_q;
			q4.push_back(q_current.coeff(3));
			q6.push_back(q_current.coeff(5));
			q4_up.push_back(q_bar.coeff(3));
			q4_down.push_back(q_under.coeff(3));
			q6_up.push_back(q_bar.coeff(5));
			q6_down.push_back(q_under.coeff(5));
			time_vector.push_back(time);
			plot_title="P2";
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 100.0;      // chose your p gain
			double kv = 50.0;      // chose your d gain
			double kvj=14.0; //joint damping
			double kpj=50.0; //joint p gain


			//desired orientation
			Eigen::Matrix3d ee_rot_mat_des;
			ee_rot_mat_des<<	cos(M_PI/3) , 0.0, sin(M_PI/3),
								0.0, 1.0, 0.0,
								-sin(M_PI/3), 0.0, cos(M_PI/3);
			
			//current orientation as rotation matrix
			Eigen::Matrix3d ee_rot_mat; //end effector rotation
			Eigen::Matrix3d rot_in_link = Matrix3d::Identity();
			robot->rotationInWorld(ee_rot_mat, link_name,rot_in_link); //or rotation
			
			//angle error
			Eigen::Vector3d ee_error;
			ee_error=-0.5*((ee_rot_mat.col(0)).cross(ee_rot_mat_des.col(0))+(ee_rot_mat.col(1)).cross(ee_rot_mat_des.col(1))+(ee_rot_mat.col(2)).cross(ee_rot_mat_des.col(2)));
			
			//J0
			Eigen::MatrixXd J0(6, robot->dof()); //end effector basic Jacobian
			robot->J_0(J0, link_name, pos_in_link);

			//Lambda 0
			Eigen::MatrixXd L0(6, 6); //Lambda_0 at end effector
			robot->taskInertiaMatrix(L0, J0);

			//nullspace
			robot->nullspaceMatrix(N, J0);

			//gravity
			robot->gravityVector(g); 

			//desired position
			Eigen::Vector3d ee_pos_des;
			ee_pos_des << 0.6, 0.3, 0.5;;


			//current position
			Eigen::Vector3d ee_pos;
			robot->position(ee_pos, link_name, pos_in_link);
			//current speed
			//speed
			Eigen::Vector3d ee_v;
			robot->linearVelocity(ee_v,link_name, pos_in_link);
			//current angular velocity
			Eigen::Vector3d w0; //end effector angular velocity 
			robot->angularVelocity(w0,link_name,pos_in_link); //w


			//decoupled force
			Eigen::VectorXd F_star;
			F_star=-1*kp*(ee_pos-ee_pos_des)-kv*(ee_v);

			//decoupled moment
			Eigen::VectorXd M_star;
			M_star=-kp*ee_error-kv*(w0);

			//total decoupled
			Eigen::VectorXd V_star(6);
			V_star<<F_star,M_star;

			//Torque
			command_torques=J0.transpose()*(L0*V_star)+N.transpose()*(-kvj*robot->_dq)+g;


			//save plot info
			x_v.push_back(ee_pos.coeff(0));
			y_v.push_back(ee_pos.coeff(1));
			z_v.push_back(ee_pos.coeff(2));
			x_v_d.push_back(ee_pos_des.coeff(0));
			y_v_d.push_back(ee_pos_des.coeff(1));
			z_v_d.push_back(ee_pos_des.coeff(2));
			dphi_x.push_back(ee_error.coeff(0));
			dphi_y.push_back(ee_error.coeff(1));
			dphi_z.push_back(ee_error.coeff(2));
			time_vector.push_back(time);
			plot_title="P3";

		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{

			double kp = 200.0;      // chose your p gain
			double kv = 30.0;      // chose your d gain
			double kvj=14.0; //joint damping
			double kpj=50.0; //joint p gain
			double V_max=0.1; //max velocity

			
			//current position
			Eigen::Vector3d ee_pos;
			robot->position(ee_pos, link_name, pos_in_link);
			//current speed
			Eigen::Vector3d ee_v;
			robot->linearVelocity(ee_v,link_name, pos_in_link);

			//desired position
			Eigen::Vector3d ee_pos_des;
			ee_pos_des << 0.6, 0.3, 0.4;

			//desired velocity
			Eigen::Vector3d ee_v_des;
			ee_v_des=kp/kv*(ee_pos_des-ee_pos);

			//nu
			double nu;
			if(V_max/ee_v_des.norm()>1.0){
				nu=1;
			}else{
				nu=V_max/ee_v_des.norm();
			}
			



			//desired joint position
			VectorXd q_des=VectorXd::Zero(dof);
			


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
			F=Lambda*(-kp*(ee_pos-ee_pos_des)-kv*(ee_v));

			//command_torques=Jv.transpose()*F +N.transpose()*robot->_M*(-kpj*(robot->_q-q_des)-kvj*robot->_dq)+g; //a
			command_torques=Jv.transpose()*Lambda*(-kv*(ee_v-nu*ee_v_des)) +N.transpose()*robot->_M*(-kpj*(robot->_q-q_des)-kvj*robot->_dq)+g; //b

			//save plot info
			x_v.push_back(ee_pos.coeff(0));
			y_v.push_back(ee_pos.coeff(1));
			z_v.push_back(ee_pos.coeff(2));
			x_v_d.push_back(ee_pos_des.coeff(0));
			y_v_d.push_back(ee_pos_des.coeff(1));
			z_v_d.push_back(ee_pos_des.coeff(2));
			v_current.push_back(ee_v.norm());
			v_max_.push_back(V_max);
			time_vector.push_back(time);
			plot_title="P4";
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

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
		
		)",
				py::globals(), locals);
	}else if(controller_number == QUESTION_2){
		// Start the Python interpreter
		py::scoped_interpreter guard{};
		using namespace py::literals;

		// Save the necessary local variables in a Python dict
		py::dict locals = py::dict{
			"x_v"_a = x_v,
			"y_v"_a = y_v,
			"z_v"_a = z_v,
			"x_v_d"_a = x_v_d,
			"y_v_d"_a = y_v_d,
			"z_v_d"_a = z_v_d,
			"time"_a = time_vector,
			"plot_title"_a=plot_title,
			"q4"_a=q4,
			"q4_up"_a=q4_up,
			"q4_down"_a=q4_down,
			"q6"_a=q6,
			"q6_up"_a=q6_up,
			"q6_down"_a=q6_down,
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
		plt.plot(time,q4,label='q4',color='r')
		plt.plot(time,q6,label='q6',color='b')
		plt.plot(time,q4_up,label='q4_up',color='r',linestyle='--')
		plt.plot(time,q6_up,label='q6_up',color='b',linestyle='--')
		plt.plot(time,q4_down,label='q4_down',color='r',linestyle='-.')
		plt.plot(time,q6_down,label='q6_down',color='b',linestyle='-.')
		plt.xlabel('Time [s]')
		plt.ylabel('Angle [rad]')
		plt.legend()
		plt.show()
		
		)",
				py::globals(), locals);
	}else if(controller_number == QUESTION_3){
		// Start the Python interpreter
		py::scoped_interpreter guard{};
		using namespace py::literals;

		// Save the necessary local variables in a Python dict
		py::dict locals = py::dict{
			"x_v"_a = x_v,
			"y_v"_a = y_v,
			"z_v"_a = z_v,
			"x_v_d"_a = x_v_d,
			"y_v_d"_a = y_v_d,
			"z_v_d"_a = z_v_d,
			"time"_a = time_vector,
			"plot_title"_a=plot_title,
			"dphi_x"_a=dphi_x,
			"dphi_y"_a=dphi_y,
			"dphi_z"_a=dphi_z,
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
		plt.suptitle(plot_title+" Orientation error")	
		plt.plot(time,dphi_x,'r',label='dphi_x')
		plt.plot(time,dphi_y,'b',label='dphi_y')
		plt.plot(time,dphi_z,'g',label='dphi_z')
		plt.xlabel('Time [s]')
		plt.ylabel('Angle Error [rad]')
		plt.legend()
		plt.show()

		
		)",
				py::globals(), locals);
	}else if(controller_number == QUESTION_4){
		// Start the Python interpreter
		py::scoped_interpreter guard{};
		using namespace py::literals;

		// Save the necessary local variables in a Python dict
		py::dict locals = py::dict{
			"x_v"_a = x_v,
			"y_v"_a = y_v,
			"z_v"_a = z_v,
			"x_v_d"_a = x_v_d,
			"y_v_d"_a = y_v_d,
			"z_v_d"_a = z_v_d,
			"time"_a = time_vector,
			"plot_title"_a=plot_title,
			"v_current"_a=v_current,
			"v_max_"_a=v_max_,
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
		plt.suptitle(plot_title+" end efector velocity")
		plt.plot(time,v_current,'r',label='v_current')
		plt.plot(time,v_max_,'b',label='v_max')
		plt.xlabel('Time [s]')
		plt.ylabel('Speed [m/s]')
		plt.legend()
		plt.show()


		
		)",
				py::globals(), locals);
	}
	return 0;
}
