#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;

#define _USE_MATH_DEFINES

class LegIK{
	private:
		double x;
		double y;
		double z;

	public:
		LegIK(const double* leg_pos){
			 x = leg_pos[0];
			 y = leg_pos[1];
			 z = leg_pos[2];
		}
		tuple<double, double> solver(double x, double z);
};


tuple<double, double> LegIK::solver(double x, double z){
	M = sqrt(x*x, z*z);
	theta1 = atan2(x,z) - acos((l2*l2-l1*l1-M*M)/(-2*M*l1));
	theta2 = M_PI - acos((M*M - l1*l1 - l2*l2)/(-2*l1*l2));

	return forward_as_tuple(theta1, theta2);

}

class Leg_controll{
	private:
		double theta1;
		double theta2;

	public:



};

int main(int argc, char **argv){
	ros::init(argc, argv, "cpg_controller");

	ros::NodeHandle n;
	
	ros::Publisher body_joint1_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/body_joint1_position_controller/command", 100);
	ros::Publisher body_joint2_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/body_joint2_position_controller/command", 100);
	ros::Publisher body_joint3_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/body_joint3_position_controller/command", 100);
	ros::Publisher body_joint4_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/body_joint4_position_controller/command", 100);
	

	ros::Publisher upper_joint1_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/upper_joint1_position_controller/command", 100);
	ros::Publisher upper_joint2_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/upper_joint2_position_controller/command", 100);
	ros::Publisher upper_joint3_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/upper_joint3_position_controller/command", 100);
	ros::Publisher upper_joint4_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/upper_joint4_position_controller/command", 100);

	ros::Rate loop_rate(10);

	while(ros::ok()){

		std_msgs::Float64 body_joint_pos;
		std_msgs::Float64 upper_joint_pos;
		body_joint_pos.data = 0.7;
		upper_joint_pos.data = -1.4;
		body_joint1_pub.publish(body_joint_pos);
		body_joint2_pub.publish(body_joint_pos);
		body_joint3_pub.publish(body_joint_pos);
		body_joint4_pub.publish(body_joint_pos);

		upper_joint1_pub.publish(upper_joint_pos);
		upper_joint2_pub.publish(upper_joint_pos);
		upper_joint3_pub.publish(upper_joint_pos);
		upper_joint4_pub.publish(upper_joint_pos);
		ros::spinOnce();

		loop_rate.sleep();
		

	}


	return 0;
}
