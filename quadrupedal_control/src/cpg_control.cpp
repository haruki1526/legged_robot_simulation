#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <algorithm>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

#define _USE_MATH_DEFINES


const double l1 =0.13, l2=0.13;

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

tuple<double, double> solver(double x, double z){
	double theta1, theta2;
	double M = sqrt(x*x + z*z);
	theta1 = -atan2(x,z) + acos((l2*l2-l1*l1-M*M)/(-2*M*l1));
	theta2 = -M_PI + acos((M*M - l1*l1 - l2*l2)/(-2*l1*l2));


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
			ktlrr = 3.3;
			ktsr = 3.0;

			Feede=0;
			Feedf=0;
			uosub = handle.subscribe("/uo_com", 10, &CPG::uoCallback, this);
		}
		double cpg(double connect_num, double yw_con_e, double yw_con_f);
		double get_ye(){
			return ye;
		}
		double get_yf(){
			return yf;
		}
		void feed(double body_pitch_angle, double body_roll_angle, double hip, double theta_o, int d_leg, double C_hip);
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
		leg_com rf_leg_com;
		leg_com rb_leg_com;
		leg_com lf_leg_com;
		leg_com lb_leg_com;
		double theta_o;
		double theta_A;
		double theta_B;
		double theta_C;
		double theta_stance;
		double rf_state;
		double rb_state;
		double lf_state;
		double lb_state;
		double A_knee;
		double B_knee;
		double C_knee;
		double A_hip;
		double B_hip;
		double C_hip;
		double f_C_hip;
		double f_C_knee;
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
		}
		void imuSubCallback(const sensor_msgs::Imu& imu_msg);
		void jointSubCallback(const sensor_msgs::JointState& joint_state_msg);
		void command(double y_rf, double y_rb, double y_lf, double y_lb);
		double get_pitch(){
			return pitch_angle;
		}
		double get_roll(){
			return roll_angle;
		}
		leg get_rf(){
			return rf_leg;
		}
		leg get_rb(){
			return rb_leg;
		}
		leg get_lf(){
			return lf_leg;
		}
		leg get_lb(){
			return lb_leg;
		}
		double get_C_hip(){
			return C_hip;
		}

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

void CPG::feed(double body_pitch_angle, double body_roll_angle, double hip, double theta_o, int d_leg, double C_hip){
	theta_vsr = hip - 0;  //body_pitch_angle
	Feede_tsr_vsr = ktsr * (theta_vsr - C_hip *(-1)); //theta_o
	Feedf_tsr_vsr = -Feede_tsr_vsr;

	Feede_tlrr = d_leg * ktlrr * body_roll_angle;
	Feedf_tlrr = -Feede_tlrr;

	Feede = Feede_tsr_vsr + 0; //Feede_tlrr;
	Feedf = Feedf_tsr_vsr + 0; //Feedf_tlrr;

}

void RoboControl::jointSubCallback(const sensor_msgs::JointState& joint_state_msg){
	rf_leg.hip = -joint_state_msg.position[0];
	rb_leg.hip = -joint_state_msg.position[1];
	lb_leg.hip = -joint_state_msg.position[2];
	lf_leg.hip = -joint_state_msg.position[3];
	rf_leg.knee = -joint_state_msg.position[4];
	rb_leg.knee = -joint_state_msg.position[5];
	lb_leg.knee = -joint_state_msg.position[6];
	lf_leg.knee = -joint_state_msg.position[7];
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

void RoboControl::command(double y_rf, double y_rb, double y_lf, double y_lb){
	tie(A_hip, A_knee) = solver(-0.08, 0.19);
	tie(B_hip, B_knee) = solver(0.00, 0.19);
	tie(C_hip, C_knee) = solver(-0.04, 0.24);
	ROS_INFO("C_knee=%f", rf_state);
	theta_o = C_hip;

	if(rf_state == 1){
		if(rf_leg.hip > -rf_leg_com.hip.data){
			rf_state=2;
		}else{
			rf_state=1;
		}
	}else if(rf_state == 2){
		rf_leg_com.hip.data = B_hip;
		rf_leg_com.knee.data = B_knee + kmy*y_rf;/////////////////////////////
		if(y_rf <= 0){
			rf_state=3;
		}
	}else if(y_rf > 0){
		rf_leg_com.hip.data = A_hip; //1.2 * rf_leg.hip;
		rf_leg_com.knee.data = A_knee; //0.8;
		rf_state = 1;
	}else if(y_rf <= 0){
		rf_leg_com.hip.data = C_hip;//theta_o + theta_stance + 0; //pitch_angle;
		rf_leg_com.knee.data = C_knee;
		rf_state = 3;
	}


	if(rb_state == 1){
		if(rb_leg.hip > -rb_leg_com.hip.data){
			rb_state=2;
		}else{
			rb_state=1;
		}
	}else if(rb_state == 2){
		rb_leg_com.hip.data = B_hip;//-0.17;
		rb_leg_com.knee.data = B_knee+ kmy*y_rb; //1.0;/////////////////////////////
		if(y_rb <= 0){
			rb_state=3;
		}
	}else if(y_rb > 0){
		rb_leg_com.hip.data = A_hip; //1.2 * rb_leg.hip;
		rb_leg_com.knee.data = A_knee; //0.8;
		rb_state = 1;
	}else if(y_rb <= 0){
		rb_leg_com.hip.data = C_hip; //theta_o + theta_stance + 0; //pitch_angle;
		rb_leg_com.knee.data = C_knee; //0.61;
		rb_state = 3;
	}


	if(lf_state == 1){
		if(lf_leg.hip > -lf_leg_com.hip.data){
			lf_state=2;
		}else{
			lf_state=1;
		}
	}else if(lf_state == 2){
		lf_leg_com.hip.data = B_hip; //-0.17;
		lf_leg_com.knee.data = B_knee+ kmy*y_lf; //1.0;/////////////////////////////
		if(y_lf <= 0){
			lf_state=3;
		}
	}else if(y_lf > 0){
		lf_leg_com.hip.data = A_hip; //1.2 * lf_leg.hip;
		lf_leg_com.knee.data = A_knee; //0.8;
		lf_state = 1;
	}else if(y_lf <= 0){
		lf_leg_com.hip.data = C_hip; //theta_o + theta_stance + 0; //pitch_angle;
		lf_leg_com.knee.data = C_knee; //0.61;
		lf_state = 3;
	}



	if(lb_state == 1){
		if(lb_leg.hip > -lb_leg_com.hip.data){
			lb_state=2;
		}else{
			lb_state=1;
		}
	}else if(lb_state == 2){
		lb_leg_com.hip.data = B_hip; //-0.17;
		lb_leg_com.knee.data = B_knee+ kmy*y_lb; //1.0;/////////////////////////////
		if(y_lb <= 0){
			lb_state=3;
		}
	}else if(y_lb > 0){
		lb_leg_com.hip.data = A_hip; //1.2 * lb_leg.hip;
		lb_leg_com.knee.data = A_knee; //0.8;
		lb_state = 1;
	}else if(y_lb <= 0){
		lb_leg_com.hip.data = C_hip; //theta_o + theta_stance + 0; //pitch_angle;
		lb_leg_com.knee.data = C_knee; //0.61;
		lb_state = 3;
	}
	
	/*rf_leg_com.hip.data = -rf_leg_com.hip.data;
	rb_leg_com.hip.data = -rb_leg_com.hip.data;
	lf_leg_com.hip.data = -lf_leg_com.hip.data;
	lb_leg_com.hip.data = -lb_leg_com.hip.data;
	rf_leg_com.knee.data = -rf_leg_com.knee.data;
	rb_leg_com.knee.data = -rb_leg_com.knee.data;
	lf_leg_com.knee.data = -lf_leg_com.knee.data;
	lb_leg_com.knee.data = -lb_leg_com.knee.data;*/

	rf_hip_pub.publish(rf_leg_com.hip);
	rf_knee_pub.publish(rf_leg_com.knee);
	rb_hip_pub.publish(rb_leg_com.hip);
	rb_knee_pub.publish(rb_leg_com.knee);
	lf_hip_pub.publish(lf_leg_com.hip);
	lf_knee_pub.publish(lf_leg_com.knee);
	lb_hip_pub.publish(lb_leg_com.hip);
	lb_knee_pub.publish(lb_leg_com.knee);

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
	double theta_o=-0.87;

	double pos_x=0;
	double pos_z=0.23;
	CPG unit_rf(1.0, 0.0, n); //ef
	CPG unit_rb(0.0, 1.0, n);
	CPG unit_lf(1.0, 0.0, n);
	CPG unit_lb(0.0, 1.0, n);
	double y_rf, y_rb, y_lf, y_lb;
	double w_lf_rf=-1.3, w_rf_lf=-1.3, w_rf_rb=-1.30, w_lb_rb=-1.3, w_rb_lb=-1.3, w_lf_lb=-1.30;
	double in_rf_e=0, in_rf_f=0, in_rb_e=0, in_rb_f=0, in_lf_e=0, in_lf_f=0, in_lb_e=0, in_lb_f=0;

  double w_rb_rf = -1.3, w_lb_lf = -1.3;

	leg rf_leg;
	leg rb_leg;
	leg lf_leg;
	leg lb_leg;

	while(ros::ok()){

		y_rf = unit_rf.cpg(0, in_rf_e, in_rf_f);
		y_rb = unit_rb.cpg(0, in_rb_e, in_rb_f);
		y_lf = unit_lf.cpg(0, in_lf_e, in_lf_f);
		y_lb = unit_lb.cpg(0, in_lb_e, in_lb_f);

		/*in_rf_f = unit_lf.get_yf() * w_rf_lf + unit_rb.get_yf() * w_rf_rb; //network for tekken
		in_rf_e = unit_lf.get_ye() * w_rf_lf + unit_rb.get_ye() * w_rf_rb;
		in_rb_f = unit_lb.get_yf() * w_rb_lb;
		in_rb_e = unit_lb.get_ye() * w_rb_lb;
		in_lf_f = unit_rf.get_yf() * w_lf_rf + unit_lb.get_yf() * w_lf_lb;
		in_lf_e = unit_rf.get_ye() * w_lf_rf + unit_lb.get_ye() * w_lf_lb;
		in_lb_f = unit_rb.get_yf() * w_lb_rb;
		in_lb_e = unit_rb.get_ye() * w_lb_rb;*/


		in_rf_f = unit_lf.get_yf() * w_rf_lf + unit_rb.get_yf() * w_rf_rb; //network for patrash
		in_rf_e = unit_lf.get_ye() * w_rf_lf + unit_rb.get_ye() * w_rf_rb;
		in_rb_f = unit_lb.get_yf() * w_rb_lb + unit_rf.get_yf() * w_rb_rf;//
		in_rb_e = unit_lb.get_ye() * w_rb_lb + unit_rf.get_ye() * w_rb_rf;//
		in_lf_f = unit_rf.get_yf() * w_lf_rf + unit_lb.get_yf() * w_lf_lb;
		in_lf_e = unit_rf.get_ye() * w_lf_rf + unit_lb.get_ye() * w_lf_lb;
		in_lb_f = unit_rb.get_yf() * w_lb_rb + unit_lf.get_yf() * w_lb_lf;//
		in_lb_e = unit_rb.get_ye() * w_lb_rb + unit_lf.get_ye() * w_lb_lf;//

		rf_leg = robo.get_rf();
		rb_leg = robo.get_rb();
		lf_leg = robo.get_lf();
		lb_leg = robo.get_lb();
		unit_rf.feed(robo.get_pitch(), robo.get_roll(), rf_leg.hip, theta_o, 1, robo.get_C_hip());
		unit_rb.feed(robo.get_pitch(), robo.get_roll(), rb_leg.hip, theta_o, 1, robo.get_C_hip());
		unit_lf.feed(robo.get_pitch(), robo.get_roll(), lf_leg.hip, theta_o, -1, robo.get_C_hip());
		unit_lb.feed(robo.get_pitch(), robo.get_roll(), lb_leg.hip, theta_o, -1, robo.get_C_hip());

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

		/*in_rf_f = unit_lf.get_yf() * w_rf_lf;
		in_rf_e = unit_lf.get_ye() * w_rf_lf;
		in_lf_f = unit_rf.get_yf() * w_lf_rf;
		in_lf_e = unit_rf.get_ye() * w_lf_rf;*/
