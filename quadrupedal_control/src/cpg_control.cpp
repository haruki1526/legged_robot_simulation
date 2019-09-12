#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

using namespace std;

#define _USE_MATH_DEFINES
double joint_state[8];

class Leg_control{
	private:
		double hip_joint_state;
		double knee_joint_state;
		
		
		ros::Publisher pub_hip_joint;
		ros::Publisher pub_knee_joint;
		ros::NodeHandle n;
		std_msgs::Float64 hip_joint_msg;
		std_msgs::Float64 knee_joint_msg;
		const double l1=0.13;
		const double l2=0.13;
		double M;

	public:
		Leg_control(string hip_joint_name, string knee_joint_name, ros::NodeHandle nodeHandle){
			n = nodeHandle;
			pub_hip_joint = n.advertise<std_msgs::Float64>("/quadrupedal_robot/"+hip_joint_name+"_position_controller/command", 100);
			pub_knee_joint = n.advertise<std_msgs::Float64>("/quadrupedal_robot/"+knee_joint_name+"_position_controller/command", 100);

		}
		void control_command();
		
		tuple<double, double> solver(double x, double z);


};

void Leg_control::control_command(){

	pub_hip_joint.publish(hip_joint_msg);
	pub_knee_joint.publish(knee_joint_msg);
}

tuple<double, double> Leg_control::solver(double x, double z){
	M = sqrt(x*x + z*z);
	hip_joint_msg.data = -atan2(x,z) + acos((l2*l2-l1*l1-M*M)/(-2*M*l1));
	knee_joint_msg.data = -M_PI + acos((M*M - l1*l1 - l2*l2)/(-2*l1*l2));


	return forward_as_tuple(hip_joint_msg.data, knee_joint_msg.data);
}


void jointSubCallback(const sensor_msgs::JointState& joint_state_msg){

	for(int i=0;i<8;i++){
		joint_state[i] = joint_state_msg.position[i];

	}

	ROS_INFO("%f", joint_state[0]);


}


int main(int argc, char **argv){
	ros::init(argc, argv, "cpg_controller");

	ros::NodeHandle n;

	ros::Subscriber sub_joint_state = n.subscribe("/quadrupedal_robot/joint_states", 10, jointSubCallback);

	

	ros::Rate loop_rate(100);
	Leg_control rf_leg("body_joint1", "upper_joint1", n);
	Leg_control rb_leg("body_joint2", "upper_joint2", n);
	Leg_control lf_leg("body_joint4", "upper_joint4", n);
	Leg_control lb_leg("body_joint3", "upper_joint3", n);

	double pos_x=0;
	double pos_z=0.23;

	while(ros::ok()){



		rf_leg.solver(pos_x, pos_z);
		rb_leg.solver(pos_x, pos_z);
		lf_leg.solver(pos_x, pos_z);
		lb_leg.solver(pos_x, pos_z);

		rf_leg.control_command();
		rb_leg.control_command();
		lf_leg.control_command();
		lb_leg.control_command();

		ros::spinOnce();

		loop_rate.sleep();
		

	}


	return 0;
}
