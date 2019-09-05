#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;

#define _USE_MATH_DEFINES

class Leg_control{
	private:
		std_msgs::Float64 theta1;
		std_msgs::Float64 theta2;
		ros::Publisher pub_theta1;
		ros::Publisher pub_theta2;
		ros::NodeHandle n;
		std_msgs::Float64 theta1_msg;
		std_msgs::Float64 theta2_msg;
		const double l1=0.13;
		const double l2=0.13;
		double M;

	public:
		Leg_control(string theta1_name, string theta2_name, ros::NodeHandle nodeHandle){
			n = nodeHandle;
			pub_theta1 = n.advertise<std_msgs::Float64>("/quadrupedal_robot/"+theta1_name+"_position_controller/command", 100);
			pub_theta2 = n.advertise<std_msgs::Float64>("/quadrupedal_robot/"+theta2_name+"_position_controller/command", 100);

		}
		void control_command();
		
		tuple<double, double> solver(double x, double z);

};

void Leg_control::control_command(){

	pub_theta1.publish(theta1);
	pub_theta2.publish(theta2);
}

tuple<double, double> Leg_control::solver(double x, double z){
	M = sqrt(x*x + z*z);
	theta1.data = -atan2(x,z) + acos((l2*l2-l1*l1-M*M)/(-2*M*l1));
	theta2.data = -M_PI + acos((M*M - l1*l1 - l2*l2)/(-2*l1*l2));


	return forward_as_tuple(theta1.data, theta2.data);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "cpg_controller");

	ros::NodeHandle n;
	

	ros::Rate loop_rate(10);
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
