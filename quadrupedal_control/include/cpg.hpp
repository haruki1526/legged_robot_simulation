#ifndef _CPG_H_INCLUDED
#define _CPG_H_INCLUDED




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

#endif




