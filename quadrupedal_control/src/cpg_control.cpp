#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <algorithm>
#include <std_msgs/Float64MultiArray.h>


//中立地点を作る

using namespace std;

#define _USE_MATH_DEFINES

const double l1 =0.13, l2=0.13;

struct leg{
	double knee;
	double hip;
	std_msgs::Float64 knee_com;
	std_msgs::Float64 hip_com;
	int state;
	double A_hip;
	double B_hip;
	double C_hip;
	double A_knee;
	double B_knee;
	double C_knee;
	double theta_o_hip;
	double theta_o_knee;
};

struct quaternion{
	double x;
	double y;
	double z;
	double w;
};

tuple<double, double> solver_f(double x, double z){
	double theta1, theta2;
	double M = sqrt(x*x + z*z);
	theta1 = -atan2(x,z) + acos((l2*l2-l1*l1-M*M)/(-2*M*l1));
	theta2 = -M_PI + acos((M*M - l1*l1 - l2*l2)/(-2*l1*l2));

	return forward_as_tuple(theta1, theta2);

}

tuple<double, double> solver_h(double x, double z){
	x = -x;
	double theta1, theta2;
	double M = sqrt(x*x + z*z);
	theta1 = -1*(-atan2(x,z) + acos((l2*l2-l1*l1-M*M)/(-2*M*l1)));
	theta2 = -1*(-M_PI + acos((M*M - l1*l1 - l2*l2)/(-2*l1*l2)));

	return forward_as_tuple(theta1, theta2);

}


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
		double theta_vsr;
		double ktsr;
		double ktlrr;
		double Feede_tsr_vsr;
		double Feedf_tsr_vsr;
		double Feede_tlrr;
		double Feedf_tlrr;
		vector<double> w;
    ros::NodeHandle handle;

		double dt;

		double Feede;
		double Feedf;
		double in_sum;
		ros::Subscriber uosub;

	public:
		CPG(double in_ue, double in_uf, ros::NodeHandle n){
      handle = n;


			tau = 0.03;
			tauv = 0.6;
			beta = 3.0; //2
			uo = 1.0;
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
			ktlrr = 3.3;
			ktsr = -3.0; //脚角度フィードバックゲイン 負の結合なのでマイナス

			Feede=0;
			Feedf=0;
			uosub = handle.subscribe("/uo_com", 10, &CPG::uoCallback, this);
		}
		double cpg(double yw_con_e, double yw_con_f);
		double get_ye(){
			return ye;
		}
		double get_yf(){
			return yf;
		}
		void feed(double body_pitch_angle, double body_roll_angle, double hip, double theta_o, int d_leg);
		//double Feede_tsr();
		//double Feedf_tsr();
    void uoCallback(const std_msgs::Float64& uo_msg);

};

class RoboControl{
	private:

		leg rf_leg;
		leg rb_leg;
		leg lf_leg;
		leg lb_leg;
		double theta_o;
		double theta_stance;
		double kmy;

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
			theta_o = -0.87;
			theta_stance = 0;
			kmy = 0.1;

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
			
			tie(rf_leg.A_hip, rf_leg.A_knee) = solver_f(-0.03, 0.20);
			tie(rf_leg.B_hip, rf_leg.B_knee) = solver_f(0.03, 0.20);
			tie(rf_leg.C_hip, rf_leg.C_knee) = solver_f(-0.02, 0.22);
			tie(rb_leg.A_hip, rb_leg.A_knee) = solver_h(-0.03, 0.20);
			tie(rb_leg.B_hip, rb_leg.B_knee) = solver_h(0.03, 0.20);
			tie(rb_leg.C_hip, rb_leg.C_knee) = solver_h(-0.02, 0.22);
			tie(lf_leg.A_hip, lf_leg.A_knee) = solver_f(-0.03, 0.20);
			tie(lf_leg.B_hip, lf_leg.B_knee) = solver_f(0.03, 0.20);
			tie(lf_leg.C_hip, lf_leg.C_knee) = solver_f(-0.02, 0.22);
			tie(lb_leg.A_hip, lb_leg.A_knee) = solver_h(-0.03, 0.20);
			tie(lb_leg.B_hip, lb_leg.B_knee) = solver_h(0.03, 0.20);
			tie(lb_leg.C_hip, lb_leg.C_knee) = solver_h(-0.02, 0.22);

			tie(rf_leg.theta_o_hip, rf_leg.theta_o_knee) = solver_f(0.00, 0.22);
			tie(rb_leg.theta_o_hip, rb_leg.theta_o_knee) = solver_h(0.00, 0.22);
			tie(lf_leg.theta_o_hip, lf_leg.theta_o_knee) = solver_f(0.00, 0.22);
			tie(lb_leg.theta_o_hip, lb_leg.theta_o_knee) = solver_h(0.00, 0.22);
		}
		void imuSubCallback(const sensor_msgs::Imu& imu_msg);
		void jointSubCallback(const sensor_msgs::JointState& joint_state_msg);
		void command(double y_rf, double y_rb, double y_lf, double y_lb);
		void phase_determine(double y, leg& target_leg);
		double get_pitch(){
			return pitch_angle;
		}
		double get_roll(){
			return roll_angle;
		}
		leg get_rf(){ return rf_leg; }
		leg get_rb(){ return rb_leg; }
		leg get_lf(){ return lf_leg; }
		leg get_lb(){ return lb_leg; }

};


double CPG::cpg(double yw_con_e, double yw_con_f){

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

void CPG::feed(double body_pitch_angle, double body_roll_angle, double hip, double theta_o, int d_leg){
	theta_vsr = hip - 0;  //body_pitch_angle
	Feede_tsr_vsr = ktsr * (theta_vsr - theta_o); //theta_o
	//ROS_INFO("feede=%f", Feede_tsr_vsr);
	Feedf_tsr_vsr = -Feede_tsr_vsr;

	Feede_tlrr = d_leg * ktlrr * body_roll_angle;
	Feedf_tlrr = -Feede_tlrr;

	Feede = Feede_tsr_vsr + 0; //Feede_tlrr;
	Feedf = Feedf_tsr_vsr + 0; //Feedf_tlrr;

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

void RoboControl::phase_determine(double y, leg& target_leg){


	if(y <= 0){	
		target_leg.hip_com.data = target_leg.C_hip;
		target_leg.knee_com.data = target_leg.C_knee;
		target_leg.state = 3;

	}else if(target_leg.state == 1 && target_leg.A_hip <=  target_leg.hip && target_leg.A_knee <=  target_leg.knee){
		target_leg.state = 2;
	}else if(target_leg.state == 2){
		target_leg.hip_com.data = target_leg.B_hip;
		target_leg.knee_com.data = target_leg.B_knee;
	}else if(y > 0){
		target_leg.hip_com.data = target_leg.A_hip;
		target_leg.knee_com.data = target_leg.A_knee;
		target_leg.state = 1;

	}


}

void RoboControl::command(double y_rf, double y_rb, double y_lf, double y_lb){

	//ROS_INFO("C_knee=%f", rf_state);
	//書き込むときは+ 読み込むときは-

	//ROS_INFO("y_rf=%f", y_rf);
	phase_determine(y_rf, rf_leg);
	phase_determine(y_rb, rb_leg);
	phase_determine(y_lf, lf_leg);
	phase_determine(y_lb, lb_leg);

	rf_hip_pub.publish(rf_leg.hip_com);
	rf_knee_pub.publish(rf_leg.knee_com);
	rb_hip_pub.publish(rb_leg.hip_com);
	rb_knee_pub.publish(rb_leg.knee_com);
	lf_hip_pub.publish(lf_leg.hip_com);
	lf_knee_pub.publish(lf_leg.knee_com);
	lb_hip_pub.publish(lb_leg.hip_com);
	lb_knee_pub.publish(lb_leg.knee_com);

}

///////////////////////////////////////dataとるため
void CPG::uoCallback(const std_msgs::Float64& uo_msg){

  uo = uo_msg.data;

}
/////////////////////////////


int main(int argc, char **argv){
	ros::init(argc, argv, "cpg_controller");

	ros::NodeHandle n;
  
	ros::Publisher cpg_pub = n.advertise<std_msgs::Float64MultiArray>("/quadrupedal_robot/cpg_state", 100);
  std_msgs::Float64MultiArray cpg_array;

	ros::Rate loop_rate(1000);

	RoboControl robo(n);

	double pos_x=0;
	double pos_z=0.23;
	CPG unit_rf(0.1, -0.1, n); //ef
	CPG unit_rb(-0.5, 0.5, n);
	CPG unit_lf(-1.0, 0.3, n);
	CPG unit_lb(0.2, -1.0, n);
	double y_rf, y_rb, y_lf, y_lb;
	double w_lf_rf=-2.0, w_rf_lf=-2.0, w_rf_rb=-0.57, w_lb_rb=-2.0, w_rb_lb=-2.0, w_lf_lb=-0.57;
	double in_rf_e=0, in_rf_f=0, in_rb_e=0, in_rb_f=0, in_lf_e=0, in_lf_f=0, in_lb_e=0, in_lb_f=0;

  double w_rb_rf = 0.0, w_lb_lf = 0.0;

	leg rf_leg;
	leg rb_leg;
	leg lf_leg;
	leg lb_leg;

	while(ros::ok()){

		y_rf = unit_rf.cpg(in_rf_e, in_rf_f);
		y_rb = unit_rb.cpg(in_rb_e, in_rb_f);
		y_lf = unit_lf.cpg(in_lf_e, in_lf_f);
		y_lb = unit_lb.cpg(in_lb_e, in_lb_f);


		in_rf_f = unit_lf.get_yf() * w_rf_lf + unit_rb.get_yf() * w_rf_rb;
		in_rf_e = unit_lf.get_ye() * w_rf_lf + unit_rb.get_ye() * w_rf_rb;
		in_rb_f = unit_lb.get_yf() * w_rb_lb + unit_rf.get_yf() * w_rb_rf;//
		in_rb_e = unit_lb.get_ye() * w_rb_lb + unit_rf.get_ye() * w_rb_rf;//
		in_lf_f = unit_rf.get_yf() * w_lf_rf + unit_lb.get_yf() * w_lf_lb;
		in_lf_e = unit_rf.get_ye() * w_lf_rf + unit_lb.get_ye() * w_lf_lb;
		in_lb_f = unit_rb.get_yf() * w_lb_rb + unit_lf.get_yf() * w_lb_lf;//
		in_lb_e = unit_rb.get_ye() * w_lb_rb + unit_lf.get_ye() * w_lb_lf;

		/*in_rf_f = unit_lf.get_yf() * w_rf_lf;
		in_rf_e = unit_lf.get_ye() * w_rf_lf;
		in_rb_f = unit_lb.get_yf() * w_rb_lb;//
		in_rb_e = unit_lb.get_ye() * w_rb_lb;//
		in_lf_f = unit_rf.get_yf() * w_lf_rf;
		in_lf_e = unit_rf.get_ye() * w_lf_rf;
		in_lb_f = unit_rb.get_yf() * w_lb_rb;//
		in_lb_e = unit_rb.get_ye() * w_lb_rb;*/

		rf_leg = robo.get_rf();
		rb_leg = robo.get_rb();
		lf_leg = robo.get_lf();
		lb_leg = robo.get_lb();
		unit_rf.feed(robo.get_pitch(), robo.get_roll(), rf_leg.hip, rf_leg.theta_o_hip, 1);
		unit_rb.feed(robo.get_pitch(), robo.get_roll(), rb_leg.hip, rb_leg.theta_o_hip, 1);
		unit_lf.feed(robo.get_pitch(), robo.get_roll(), lf_leg.hip, lf_leg.theta_o_hip, -1);
		unit_lb.feed(robo.get_pitch(), robo.get_roll(), lb_leg.hip, lb_leg.theta_o_hip, -1);

		robo.command(y_rf, y_rb, y_lf, y_lb);

		ROS_INFO("rf=%f, lb=%f, lf=%f, rb=%f", y_rf, y_lb, y_lf, y_rb);

    cpg_array.data.resize(4);
    cpg_array.data[0] = y_rf;
    cpg_array.data[1] = y_rb;
    cpg_array.data[2] = y_lf;
    cpg_array.data[3] = y_lb;
    cpg_pub.publish(cpg_array);

		ros::spinOnce();

		loop_rate.sleep();

	}


	return 0;
}

