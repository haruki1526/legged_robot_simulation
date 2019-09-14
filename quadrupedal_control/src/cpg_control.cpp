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
		double tau;
		double tauv;
		double beta;
		double uo;
		double ue;
		double due;
		double ye;
		double uf;
		double duf;
		double yf;
		double wfe;
		double ve;
		double dve;
		double vf;
		double dvf;
		double y;
		vector<double> w;

		double dt;

		double Feede;
		double Feedf;
		double in_sum;

	public:
		CPG(double in_ue, double in_uf){


			tau = 0.05;
			tauv = 0.6;
			beta = 3.0;
			uo = 3.0;
			ue = in_ue;
			due = 0.0;
			ye = 1.0;
			uf = in_uf;
			duf = 0.0;
			yf = 0.0;
			wfe = -2.0;
			ve = 0;
			dve = 0;
			vf = 0;
			dvf = 0;
			dt = 0.001;

			Feede=0;
			Feedf=0;
		}
		double cpg(double connect_num, double yw_con_e, double yw_con_f);
		double get_ye(){
			return ye;
		}
		double get_yf(){
			return yf;
		}
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

	ve = ve + dve *dt;
	ue = ue + due * dt;

	dvf = (-vf + yf)/tauv;
	duf = (-uf+ wfe * ye - beta*vf + uo + Feedf + yw_con_f)/tau;

	vf = vf + dvf * dt;
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
	//ROS_INFO("pitch=%f", pitch_angle);
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

	ros::Rate loop_rate(1000);
	/*Leg_control rf_leg("body_joint1", "upper_joint1", n);
	Leg_control rb_leg("body_joint2", "upper_joint2", n);
	Leg_control lf_leg("body_joint4", "upper_joint4", n);
	Leg_control lb_leg("body_joint3", "upper_joint3", n);*/

	RoboControl robo(n);

	double pos_x=0;
	double pos_z=0.23;
	CPG unit_rf(1.0, 0.0); //ef
	CPG unit_rb(0.0, 1.0);
	CPG unit_lf(1.0, 0.0);
	CPG unit_lb(0.0, 1.0);
	double y_rf, y_rb, y_lf, y_lb;
	double w_lf_rf=-0.7, w_rf_lf=-0.7, w_rf_rb=-0.01, w_lb_rb=-0.7, w_rb_lb=-0.7, w_lf_lb=-0.01;
	double in_rf_e=0, in_rf_f=0, in_rb_e=0, in_rb_f=0, in_lf_e=0, in_lf_f=0, in_lb_e=0, in_lb_f=0;

	int count =0;
	while(ros::ok()){

		count++;
		y_rf = unit_rf.cpg(0, in_rf_e, in_rf_f);
		y_rb = unit_rb.cpg(0, in_rb_e, in_rb_f);
		y_lf = unit_lf.cpg(0, in_lf_e, in_lf_f);
		y_lb = unit_lb.cpg(0, in_lb_e, in_lb_f);
		



		/*in_rf_f = unit_lf.get_yf() * w_rf_lf;
		in_rf_e = unit_lf.get_ye() * w_rf_lf;
		in_lf_f = unit_rf.get_yf() * w_lf_rf;
		in_lf_e = unit_rf.get_ye() * w_lf_rf;*/


		in_rf_f = unit_lf.get_yf() * w_rf_lf + unit_rb.get_yf() * w_rf_rb;
		in_rf_e = unit_lf.get_ye() * w_rf_lf + unit_rb.get_ye() * w_rf_rb;
		in_rb_f = unit_lb.get_yf() * w_rb_lb;
		in_rb_e = unit_lb.get_ye() * w_rb_lb;
		in_lf_f = unit_rf.get_yf() * w_lf_rf + unit_lb.get_yf() * w_lf_lb;
		in_lf_e = unit_rf.get_ye() * w_lf_rf + unit_lb.get_ye() * w_lf_lb;
		in_lb_f = unit_rb.get_yf() * w_lb_rb;
		in_lb_e = unit_rb.get_ye() * w_lb_rb;

		ROS_INFO("rf=%f, lb=%f, lf=%f, rb=%f", y_rf, y_lb, y_lf, y_rb);
		ros::spinOnce();

		loop_rate.sleep();
		/*if(count > 1300){
			return 0;
		}*/
		

	}


	return 0;
}

