#include <ktl.h>
#include <libts01.h>
#include "arm.h"
#include "sim.h"

using namespace Ktl::Pneumatics;

#define NOISE_X 0.001
#define NOISE_P 0.04
#define NOISE_FUNC 0.5*sin(60*2*PI*t)

static const double BIT[6] = {
  21, 18, 18, 19, 17, 19
};
unsigned int res(int n){
  //printf("%d\n",0xffffffff>>(32-n));
  return 0xffffffff >> (32-n);
}
static const double RQE[DOF] = { //kannsetu enc 
 -1.0, 
 -1.0 * 0.583333,
 -1.0 * 0.56,
 -1.0,
  1.0,
 -1.0
};
static const double RE[DOF] = { //エンコーダ自体の分解能
  2*PI / res(BIT[0]),
  2*PI / res(BIT[1]),
  2*PI / res(BIT[2]),
  2*PI / res(BIT[3]),
  2*PI / res(BIT[4]),
  2*PI / res(BIT[5]),
};

double shift_angle_range(double theta){ //0~2PIのレンジに変換する
  return  theta > 0 ?  theta : theta + 2*PI;
}

const double qoffset[6] = { 
  0.0,
  21.5 / DEG,  39.0 / DEG, //1,2はリセットできない
  0.0, 0.0, 0.0
};




/******************************************************************
 *    シミュレーション用 スレッド
*******************************************************************/
//void* Simulator::thread(void* p_arg){
int main(){
  Ktl::ManagedProcess mp("av3_sim");
  Simulator* sim = new ("sshm") Simulator();  
  TS01Simulator ts01;
  ts01.create_command_server();
  ts01.create_autosampling_server();
  
  RTTask sim_task;  
  sim_task.init();
  int command;

  int ret;
  int start;

  double t;

  double voltage[PAXIS];
  const int period = 500;

  double dt = 1e-6*period; // [sec]
  Actuator::setdt( dt );
  unsigned long count=0;

  Ktl::Pneumatics::Air supply;
  Ktl::Pneumatics::Actuator act[PAXIS];
  Ktl::Pneumatics::ServoValve valve[PAXIS];
  Ktl::Vector<ADOF> q;
  Ktl::Vector<ADOF> qd;
  Ktl::Vector<ADOF> qdd;
  Ktl::Vector<ADOF> tau;

  double J1[] = {1.0, 1.0, -1.0, 1.0 };

  Ktl::Vector<3> Pp0(400.0, 0.0, 200.0);

  sim->Pp = Pp0;

  Ktl::Pneumatics::ActShape vane10S( Ktl::Pneumatics::KURODA_PRNA10S,  
				   270.0/180.0*M_PI);
  Ktl::Pneumatics::ActShape vane3S( Ktl::Pneumatics::KURODA_PRNA10S,  
				    270.0/180.0*M_PI);
  Ktl::Pneumatics::ActShape vane30S( Ktl::Pneumatics::KURODA_PRNA10S,
				     270.0/180.0*M_PI);
  Ktl::Pneumatics::ActShape CJ2_16( Ktl::Pneumatics::SMC_CJ2_16, 45.0);
  Ktl::Pneumatics::ActShape CJ2_20( Ktl::Pneumatics::SMC_CJ2_16, 45.0);

  //vaneS20.A1 *= 1.75;
  vane3S.A1  *= 0.32;
  vane30S.A1 *= 3.5;

  CJ2_20.A1 *= pow( 20.0/16.0, 2 );
  CJ2_20.A2 *= pow( 20.0/16.0, 2 );

  CJ2_16.A1 *= 2; //dual configuration
  CJ2_16.A2 *= 2; 

  CJ2_20.A1 *= 2; //dual configuration
  CJ2_20.A2 *= 2; 

  act[0].setup( vane30S );
  act[0].setInertia(  20.0 ); //[g m^2] == [1e-3*kgm^2]
  act[0].setFriction( 40, 120.0, 120.1, 0.01);

  act[1].setup( CJ2_20 );
  act[1].setInertia( 0.2 );
  act[1].setFriction( 0.6, 1.8, 1.8, 0.1);
  //act[1].setWorkingRange( -50.0, 50.0 );

  act[2].setup( CJ2_16 );
  act[2].setInertia( 0.1 );
  act[2].setFriction( 0.5, 1.2, 1.2, 0.1);

  act[3].setup(  vane3S  );  
  act[3].setInertia(  1.0 ); // [g m^2]
  act[3].setFriction( 20.0, 30.0, 30.0, 0.01);

  for(int j=0;j<PAXIS;j++){ 
    valve[j].connect( supply, P_PORT );
    valve[j].connect( act[j].room1, A_PORT );
    valve[j].connect( act[j].room2, B_PORT );
    valve[j].setup();
  }

  supply.init();
  supply.P = 600.0;//msg.Ps;

  for(int j=0;j<PAXIS;j++){
    act[j].init();
    valve[j].set(5.0);
  }

  act[0].position =  0.0/DEG / J1[0];;
  //act[1].position = 60.0/DEG / J1[1];

  sim->arm.m[0]  = 1.0 * 1e-3; //[Mg]
  sim->arm.Ig[0] = 0.25*1e3;//[g m^2] = [Mg mm^2]
  sim->arm.m[1]  = 1.0 * 1e-3; //[Mg]
  sim->arm.Ig[1] = 0.2*1e3;//[g m^2] = [Mg mm^2]
  sim->arm.m[2]  = 0.1 * 1e-3; //[Mg]
  sim->arm.Ig[2] = 0.005*1e3;//[g m^2] = [Mg mm^2]
  //sim->arm.m[2] += 1.0*1e-3;
  //sim->arm.Ig[2] += 1.0*1e-3 * sim->arm.l0[2].abs2();
  
  sim->arm.init();

  //Ktl::SliderCrankB scm; //Slider Crank Mechanism
  //scm.setupLink( 165.0, 85.0 );

  const double h[2] = {215.0, 180.0};
  const double r[2] = {39.0, 32.0}; 
  const double d[2] = {6.0, 5.0};

  double h2[2];
  double alpha[2];    

  Ktl::SliderCrankB scm[2]; //Slider Crank Mechanism  

  for (int j = 0; j < 2; j++) {
    h2[j] = sqrt(h[j] * h[j] + d[j] * d[j]);
    alpha[j] = atan2(d[j] , h[j]); 
    scm[j].setupLink(r[j], h2[j]);
  }

  double u[PAXIS];


  
  rt_print("sim module : start\n");
  sim_task.start(period);

  //while (1) {
  while (!mp.termination_requested()) {
    
    if( sim_task.receive(&command) ){
      switch(command) {
      default:
	rt_print("sim thread : unknown command %d\n",command);
	//return 0;
      }
    }
    
    t = count*dt;
    
    supply.P = ts01.output.dout[0] ? 101.0 : 600.0;
    for(int j=0;j<PAXIS;j++)
      valve[j].set(ts01.output.u[j]);

    /*
    tau_ext = ~robot.Jp() * fext; //関節外力
    for(int j=0;j<5;j++) //シリンダ外力
      Fext[j] = tau_ext[j] * J1[j];
    Fext[2] *= -1;
    */
    //act[1].ext_force = -11; //簡易重力

    //for(int j=0;j<3;j++)
    //act[j].ext_force = sgn[j]*tau[j];
  
    
    act[2].ext_force = - J1[2] *(200* 2.0 * cos(q[2]) );

    for(int j=0;j<PAXIS;j++){ 
        //act[j].ext_force = Fext[j];
      act[j].update_derivatives();
      act[j].update_state();
    }

    q[0]   = J1[0] * act[0].position;
    qd[0]  = J1[0] * act[0].velocity;
    qdd[0] = J1[0] * act[0].acceleration;

    q[1] = scm[0].forward( act[1].position ) + alpha[0];
    J1[1] =  scm[0].J;
    qd[1]  = J1[1] * act[1].velocity;
    qdd[1] = J1[1] * act[1].acceleration;

    q[2] = - scm[1].forward( act[2].position ) + alpha[1] ;
    J1[2] = - scm[1].J;
    qd[2]  = J1[2] * act[2].velocity;
    qdd[2] = J1[2] * act[2].acceleration;

    sim->arm.setq(q);

    sim->arm.q(5)  = J1[3]*act[3].position;// encoder    


    sim->arm.forward_kinematics();
    //tau = sim->arm.inverse_dynamics(qdd,qd);

    sim->Pp = Pp0;

    //Pp[0] += 6.0*sin(2*PI*0.05*t);
    //Pp[1] += 4.0*sin(2*PI*0.1*t);
    sim->Pp[2] += 7.0*sin(2*PI*0.3*t);

    //sim->arm.Pr().print();
    Ktl::Vector<3> d = sim->Pp - sim->arm.P();
    //Pp.print();
    //d.print(2);
    d.normalize();
    static const Ktl::Matrix<3, 3> E45(Ktl::Y, 45.0 / DEG);

    Ktl::Matrix<3,3> E = Ktl::Matrix<3,3>(Ktl::Z,sim->arm.q(0)) * E45;

    Ktl::Vector<3> a = ~E * d;
    sim->arm.q(4) = atan2( -a[2], sqrt(a[0]*a[0]+a[1]*a[1]) );
    sim->arm.q(3) = atan2(  a[1], a[0] );

    //robot.setq(q);
    //robot.forward_kinematics();
    //fext = -wall.calc_force(robot.Pr(),Ktl::o(),robot.Rr());

    //sim->armFK.q[j] = RQ[j] * enc[j] - qoffset[j];
    for(int j=0;j<DOF;j++){
      ts01.input.ssi[j] = shift_angle_range(
					    (sim->arm.q(j)+qoffset[j])/RQE[j] ) / RE[j];
      ts01.input.ssi[j] = ts01.input.ssi[j] << 1;
    }
    
    for(int j=0;j<PAXIS;j++){
      ts01.input.v[2*j  ] = (act[j].room1.P -101)/250.0 + 1.0;
      ts01.input.v[2*j+1] = (act[j].room2.P -101)/250.0 + 1.0;
    }

    count++;
    sim_task.wait();
  }
  
  Simulator::operator delete( sim, "sshm" ); 
  printf("sim_module done\n");
  return 0;
}
