#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <algorithm>

using namespace std;

#define _USE_MATH_DEFINES

struct leg{
	double knee;
	double hip;
};

struct leg_com{

	std_msgs::Float64 knee;
	std_msgs::Float64 hip;

};
	

struct quaternion{
	double x;
	double y;
	double z;
	double w;
};

class CPG{

	private:
		double tau = 0.05;
		double tauv = 0.6;
		double beta = 3.0;
		double uo = 1.0;
		double ue = 0.1;
		double due = 0.0;
		double ye = 0.1;
		double uf = 0.0;
		double duf = 0.0;
		double yf = 0.0;
		double wfe = -2.0;
		double ve = 0;
		double dve = 0;
		double vf = 0;
		double dvf = 0;
		double y;
		vector<double> w;

		double dt = 0.001;

		double Feede=0;
		double Feedf=0;
		double in_sum;

	public:
		double cpg(double connect_num, double yw_con_e, double yw_con_f);
		//double Feede_tsr();
		//double Feedf_tsr();

};

class RoboControl{
	private:

		leg rf_leg;
		leg rb_leg;
		leg lf_leg;
		leg lb_leg;

		quaternion orientation;

		double roll_angle;
		double pitch_angle;

		ros::NodeHandle n;
		ros::Subscriber imusub;
		ros::Subscriber jointsub;
		ros::Publisher rf_hip_pub;
		ros::Publisher rb_hip_pub;
		ros::Publisher lf_hip_pub;
		ros::Publisher lb_hip_pub;
		ros::Publisher rf_knee_pub;
		ros::Publisher rb_knee_pub;
		ros::Publisher lf_knee_pub;
		ros::Publisher lb_knee_pub;

	public:
		RoboControl(ros::NodeHandle nodeHandle){
			n = nodeHandle;

			imusub = n.subscribe("/imu", 10, &RoboControl::imuSubCallback, this);
			jointsub = n.subscribe("/quadrupedal_robot/joint_states", 10, &RoboControl::jointSubCallback, this);

			rf_hip_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/body_joint1_position_controller/command", 100);
			rb_hip_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/body_joint2_position_controller/command", 100);
			lb_hip_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/body_joint3_position_controller/command", 100);
			lf_hip_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/body_joint4_position_controller/command", 100);
			rf_knee_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/upper_joint1_position_controller/command", 100);
			rb_knee_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/upper_joint2_position_controller/command", 100);
			lb_knee_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/upper_joint3_position_controller/command", 100);
			lf_knee_pub = n.advertise<std_msgs::Float64>("/quadrupedal_robot/upper_joint4_position_controller/command", 100);
		}
		void imuSubCallback(const sensor_msgs::Imu& imu_msg);
		void jointSubCallback(const sensor_msgs::JointState& joint_state_msg);
		void command(leg_com rf_leg_com, leg_com rb_leg_com, leg_com lf_leg_com, leg_com lb_leg_com);

};



double CPG::cpg(double connect_num, double yw_con_e, double yw_con_f){

	dve = (-ve + ye)/tauv;
	due = (-ue + wfe * yf - beta*ve + uo + Feede + yw_con_e)/tau;

	ue = ue + due * dt;

	dvf = (-vf + yf)/tauv;
	duf = (-uf+ wfe * ye - beta*vf + uo + Feedf + yw_con_f)/tau;

	uf = uf + duf * dt;

	ye = max(ue, 0.0);
	yf = max(uf, 0.0);

	y = -ye + yf;

	return y;

}


void RoboControl::jointSubCallback(const sensor_msgs::JointState& joint_state_msg){
	rf_leg.hip = joint_state_msg.position[0];
	rb_leg.hip = joint_state_msg.position[1];
	lb_leg.hip = joint_state_msg.position[2];
	lf_leg.hip = joint_state_msg.position[3];
	rf_leg.knee = joint_state_msg.position[4];
	rb_leg.knee = joint_state_msg.position[5];
	lb_leg.knee = joint_state_msg.position[6];
	lf_leg.knee = joint_state_msg.position[7];
	//ROS_INFO("lf_leg_knee=%f", lf_leg.knee);

}

void RoboControl::imuSubCallback(const sensor_msgs::Imu& imu){
	orientation.x = imu.orientation.x;
	orientation.y = imu.orientation.y;
	orientation.z = imu.orientation.z;
	orientation.w = imu.orientation.w;

	pitch_angle = atan2(2*orientation.x*orientation.z + 2*orientation.w*orientation.y, 1 - 2*orientation.y*orientation.y - 2*orientation.z*orientation.z);
	roll_angle = atan2(2*orientation.y*orientation.z - 2*orientation.w*orientation.x, 1-2*orientation.x*orientation.x - 2*orientation.z*orientation.z);
	ROS_INFO("pitch=%f", pitch_angle);
}

void RoboControl::command(leg_com rf_leg_com, leg_com rb_leg_com, leg_com lf_leg_com, leg_com lb_leg_com){
	rf_hip_pub.publish(rf_leg_com.hip);
	rf_knee_pub.publish(rf_leg_com.knee);
	rb_hip_pub.publish(rb_leg_com.hip);
	rb_knee_pub.publish(rb_leg_com.knee);
	lf_hip_pub.publish(lf_leg_com.hip);
	lf_knee_pub.publish(lf_leg_com.knee);
	lb_hip_pub.publish(lb_leg_com.hip);
	lb_knee_pub.publish(lb_leg_com.knee);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "cpg_controller");

	ros::NodeHandle n;


	

	ros::Rate loop_rate(100);
	/*Leg_control rf_leg("body_joint1", "upper_joint1", n);
	Leg_control rb_leg("body_joint2", "upper_joint2", n);
	Leg_control lf_leg("body_joint4", "upper_joint4", n);
	Leg_control lb_leg("body_joint3", "upper_joint3", n);*/

	RoboControl robo(n);

	double pos_x=0;
	double pos_z=0.23;
	CPG control;

	while(ros::ok()){



		

		ros::spinOnce();

		loop_rate.sleep();
		

	}


	return 0;
}

