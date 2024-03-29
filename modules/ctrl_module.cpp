/*********************************************************************
 *   RTLinux module for the control of pneumatic cylinders
 *                                written by Kotaro Tadano  since 2003
 *********************************************************************/
//#include <ktl.h>
#include <libts01.h>
#include "controller.h"
#include "packet.h"
#include "def.h"
//#include <ms.h>
//#define RF

//static RTTask task;

/*
Ktl::Range<double> tau_range[PAXIS] = { 
  Ktl::Range<double>(-9000.0, 9000.0),
  Ktl::Range<double>(-9000.0, 9000.0), 
  Ktl::Range<double>(-8500.0, 8500.0), 
  Ktl::Range<double>(-1000.0, 1000.0)
};

static Ktl::Range<double> position_range[PAXIS] = { 
  Ktl::Range<double>(-275.0 / DEG, 275.0 / DEG),
  Ktl::Range<double>(-35.0 / DEG,37.0 / DEG),
  Ktl::Range<double>(-45.0 / DEG, 50.0 / DEG), //b2
  Ktl::Range<double>(-180.0 / DEG, 180.0 / DEG) 
};

static Ktl::Range<double> velocity_range[PAXIS] = { 

  Ktl::Range<double>(-35.0 / DEG,37.0 / DEG),
  Ktl::Range<double>(-45.0 / DEG, 50.0 / DEG), //b2
  Ktl::Range<double>(-180.0 / DEG, 180.0 / DEG) 
};

static Ktl::Range<double> acceleration_range[PAXIS] = { 
  Ktl::Range<double>(-275.0 / DEG, 275.0 / DEG),
  Ktl::Range<double>(-35.0 / DEG,37.0 / DEG),
  Ktl::Range<double>(-45.0 / DEG, 50.0 / DEG), //b2
  Ktl::Range<double>(-180.0 / DEG, 180.0 / DEG) 
};
*/

//      armFK.q[j] = RQ[j] * enc[j] - qoffset[j];
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

const double qoffset[DOF] = { 
  0.0,
  21.5 / DEG,  39.0 / DEG, //1,2はリセットできない
  //-20.0/DEG,
  0.0,
  0.0, 0.0
};

double shift_range(double angle){ //0〜２PIを-PI〜PIに変換する
  if( angle < PI )
    return angle;
  else
    return angle - 2*PI;
}

TS01 ts01;


/*******************************************************************
 *    鉗子マニピュレータ制御スレッド
 *******************************************************************/
void* Controller::thread(void *arg) {
  task.init();
  
  int ret = 0;
  unsigned long count = 0;
  int command;

  LogBuf buf;  //通信用バッファ
  
  Ktl::FIFO fifo_log("av3_log");
  fifo_log.open(Ktl::FIFO::NON_BLOCKING); //non-blocking
  
  Ktl::FIFO fifo_master("av3_master");//from gyroscope
  fifo_master.open(Ktl::FIFO::NON_BLOCKING); //non-blocking
  
  RBuf ubuf;

  Ktl::Timer timer;
  
  TS01InputData input;
  TS01OutputData output;

  Ktl::Vector<3> gqpre;
  Ktl::Vector<3> Ppre;
  
  Ktl::Vector<3> dq;
  Ktl::Vector<3> qpre;
  
  Ktl::Vector<3> qdref3_1;
  Ktl::Vector<3> qdref3_2;
  
  const double shifting_time = 0.4;
  bool flag_vtv = false;
  bool flag_stopping = false;  
  //-------------------------------------------

  const double pt = 2.0; //preparation time [s]
  double time_startup = pt;
  
  Ktl::Vector<3> Pppre;
  Ktl::Vector<3> Prpre;
  
  //period = 500;
  period = 1000;
  dt = 1e-6 * period;
  Ktl::Dynamics::setdt(dt); // sim_moduleと干渉しない

  emergency_flag = false;
  
  //制御変数の初期化
  Pd.zero();
  fext.zero();
  mext.zero();
  Pdref.zero(); //先端の目標速度xyz
  wref.zero(); //先端の目標角速度wx wy wz
  
  pjoint[0].setup( 7.350 ); //PRNA30S
  pjoint[1].setup( 2 * 314e-3, 2 * 263e-3 ); //dual cylinder
  pjoint[2].setup( 2 * 201e-3, 2 * 181e-3 ); //dual cylinder
  pjoint[3].setup( 0.6 ); //PRNA3S (不正確）

  //空気圧アクチュエータ制御変数の初期化
  for (int j = 0; j < PAXIS; j++) {
    pjoint[j].motion_range = position_range[j];
    pjoint[j].tau_range = tau_range[j];
    pjoint[j].setup_parameters();
  }
  
  double exw = exp(-dt * 2 * PI * 3.0); //ジャイロローパス
  
  //--------------------------------------------------------
  Ktl::Range<double> drange( 200.0, 700.0 );
  Pp = Ktl::Vector<3>(400.0, 600.0, -400.0);
  Ktl::Vector<3> n;
  
  double ip = 0;

  const double h[2] = {215, 180};
  const double r[2] = {39, 32}; 
  const double d[2] = {6.0, 5.0};

  double h2[2];
  double alpha2[2];    
  for (int j = 0; j < 2; j++) {
    h2[j] = sqrt(h[j] * h[j] + d[j] * d[j]);
    alpha2[j] = atan2(d[j] , h[j]);
    //printf("h2 = %f, alpha2 = %f\n",h2[j], alpha2[j]*DEG);
  }

  Ktl::SliderCrankB scm[2]; //Slider Crank Mechanism
  scm[0].setupLink(r[0], h2[0]);
  scm[1].setupLink(r[1], h2[1]);
  

  //--------------------------------------------------------------------
  
  //目標値の運動学の初期設定
  //先端位置の初期位置を設定（必須、0ではない！）
  armIK.init();
  armFK.init();
  armIK.q(1) = 30.0 / DEG;
  armFK.forward_kinematics();
  P0 = armFK.P();  
  for (int j = 0; j < PAXIS; j++){
    pjoint[j].setup_parameters();
    pjoint[j].init();
  }
  pjoint[0].J1 = 1.0; //linear ベーンモータ
  pjoint[3].J1 = 1.0;

  const Ktl::Vector<3> g2(0.0, 0.0, -9.8); //m/s^2
  Ktl::Vector<3> Ws; //スコープ重力
  Ktl::Vector<3> tau_w;
  const double dg = 230.0;// ジンバル回転中心から内視鏡重心までの距離

  Ktl::Matrix<3,3> Jp3;
  Ktl::Matrix<3,3> Jr3;


  double fp = 0; //frequency for stepper motor[Hz]
#ifndef RF
  double fpmax = 100.0e3; //[Hz] for Tokyo Tech
#else
  double fpmax = 10.0e3; //[Hz] for RF
#endif
  bool flag_stepping = false;
  bool flag_stopping2 = false;  
  Ktl::PPTrajectory<double> fpt;
  bool flag_fpt=false;

  bool operation_mode_pre;
  int passive_count = 0;
  int active_count = 0;
  double tau_fb_temp[PAXIS];

  task.wait_message(false);
  ts01.start_sampling(period);

  state = STATE_SERVO_OFF;
  flag_ev_open = false;

  while (1) {
    //while (!task.termination_requested()) {
    if (task.receive(&command)) {
      //printf("ctrl_module : received command %s\n", command_text[command]);
      
      switch (command) {
      case COMMAND_START: //--------------
	if( state != STATE_STANDBY ) break;
	if( ctrl_mode == MODE_TASK ){
	  state = STATE_TASK_RUNNING;
	  flag_feedback = false;
	}
	else{
	  state = STATE_TEST_RUNNING;
	  flag_feedback = true;
	}
	
	flag_error = false;
	operation_trigger = 0;
	
	//制御パラメータの再設定
	
	for (int j = 0; j < 3; j++) {
	  armFK.m[j] = m[j] * 1e-3; //[kg]->[Mg]
	  armFK.Ig[j] = Ig[j] * 1e3; //[kg m^2] -> [g m^2] = [Mg mm^2]
	}
	
	armFK.m[2]  += Mh * 1e-3;   
	armFK.Ig[2] += Mh * 1e-3 * armFK.l0(2).abs2();
        
	//Ws = Ms * g2;
	
	for (int j = 0; j < PAXIS; j++) {  //======= Be carefule =======
	  //Qwave[j].freq *= 0.1;
	  pjoint[j].fc_qd = fc_qd;
	  pjoint[j].fc_qdd= fc_qdd;
	  pjoint[j].fc    = fc_u;
	  pjoint[j].setup_parameters();
	  pjoint[j].Qd.set_cutoff_frequency(0.1*fc_qdd);
	  //pjoint[j].init();
	  pjoint[j].DPController::init(); //重要
	  pjoint[j].fr.friction_model = 0;
	  //pjoint[j].fr.friction_model = 1;
	}


	Pd.zero();
	fext.zero();
	mext.zero();
	Pdref.zero();	//先端の目標速度xyz
	wref.zero();	//先端の目標角速度wx wy wz
	
	//外力推定用フィルタ時定数
	ex_f = exp(-dt * 2 * PI * fc_Fext);
	distance = 450;
	operation_mode = OP_DISABLE;

	time_startup = time + pt;

	for (int j = 0; j < PAXIS; j++)
	  pjoint[j].qref = pjoint[j].q;

	break;
      case COMMAND_STOP:
	if( state == STATE_SERVO_OFF ) break;
	state = STATE_STANDBY;
	break;
      case COMMAND_SERVO_ON:
	if( state != STATE_SERVO_OFF ) break;
	state = STATE_STANDBY;
	flag_ev_open = true;
	break;
      case COMMAND_SERVO_OFF:
	state = STATE_SERVO_OFF;
	flag_ev_open = false;
	break;
      case -1:
	//rt_print("ctrl thread : terminated%d\n", command);
	break;
	
      default:
	rt_print("ctrl thread : unknown command %d\n", command);
	//return 0;
      }
    } //-----------------------------------------------------------
    
    time = count * dt; // 時刻の更新

    //if( ti > dt*1.5 )
    //printf("interval %.2f [ms] at %ld\n", ti*1e3, count);
    double ti = timer.get_time();
    timer.reset();
    ts01.read_autosampling_data(&input);
    double ts = timer.get_time();
    
    for (int j = 0; j < ADOF; j++){
      int enc = input.ssi[j] >> 1;
      armFK.q(j) = RQE[j] * shift_range( RE[j] * enc ) - qoffset[j];
    }
    
    armFK.forward_kinematics();
    Rs = armFK.R() * armFK.Tw; //内視鏡姿勢行列m
    //armFK.R().print();
    //armFK.Tw.print();
    //Rs.print(2);
    
    Ktl::Matrix<3,ADOF> Jp = armFK.Jp();
    for(int j=0;j<3;j++)
      Jp3.column(j, Jp.column(j) );

    Ktl::Matrix<3,ADOF> Jr = armFK.Jr();
    for(int j=0;j<3;j++) //ジンバルによる姿勢ヤコビ行列
      Jr3.column(j, Jr.column(j+3) );

    
    for(int j=0;j<PAXIS;j++){ // ����
      P1[j] = 250*(input.v[2*j]  - 1.0);//vP1n[j]);
      P2[j] = 250*(input.v[2*j+1]- 1.0);//vP2n[j]);
    }

    for (int j = 0; j < 3; j++) //一般化駆動力
      pjoint[j].update(armFK.q(j), P1[j], P2[j]);
    pjoint[3].update(armFK.q(5), P1[3], P2[3]);

    scm[0].inverse(   armFK.q(1)-alpha2[0] );
    pjoint[1].J1 = scm[0].J;

    scm[1].inverse( -(armFK.q(2)-alpha2[1]) );
    pjoint[2].J1 = -scm[1].J;
    

    Ms += dM;
    Ktl::limitRange(&Ms, 0.0, 3.0);
    Ws = Ms * g2;
    
    Ktl::Vector<3> f = Mh * g2;//手先が受ける力

    if( flag_scope_mount ){
      Ktl::Vector<3> fp = dg/distance *( Ws - (n&Ws)*n ); //ポートから受ける力
      f += Ws - fp; //手先が受ける力
    }

    tau_w = ~Jp3 * f;

    Ktl::Vector<3> qd3;
    for (int j = 0; j < 3; j++)
      qd3[j] = pjoint[j].qd;
    
    Pd = Jp3 * qd3;


    //--- このアルゴリズムはかなりよさげ
    //n = armFK.Rr().column(0);
    n = Rs.column(0);
    //if( !flag_pivot_lock )
    Pp = armFK.P() + distance * n;
    //Pp = armFK.P() + distance * armFK.R().column(0);//n;
    
    Ktl::Vector<3> dPr = armFK.P()-Prpre;
    Ktl::Vector<3> dPp = Pp - Pppre;
    Ktl::Vector<3> dPrn = dPr | n;
    Ktl::Vector<3> dPpn = dPp | n;
    ip = 0.995 * ip + 0.005 * ( ( dPr - dPrn ) & ( dPp - dPpn ));
    //d1 = ip;//表示のため
    Ktl::limitRange(&ip, -0.005, 0.005); 
    distance += 10*ip - (dPr & n);
    drange.limit(distance);

    Prpre = armFK.P();
    Pppre = Pp;


    //------------------------------------------------------------
    if(state <= STATE_STANDBY ) { //-------     //standby:待機
      Pref = armFK.P();

      for (int j = 0; j < PAXIS; j++) {
	if (pjoint[j].ptp.is_progressing(time))
	  pjoint[j].qref = pjoint[j].ptp.get(time);
	else
	  pjoint[j].qref = pjoint[j].q;
	
	pjoint[j].Qref.update();
      }

      for (int j = 0; j < PAXIS; j++){
	pjoint[j].tau_ref = 0.0;
      }
      for (int j = 0; j < 3; j++){
	pjoint[j].tau_ref = pjoint[j].calcZ(-1,0,0);
      }
      pjoint[1].tau_ref = -pjoint[1].K * pjoint[1].q;
      pjoint[2].tau_ref = pjoint[2].K * sin( 1.2*pjoint[2].q + 0.7 );
      for (int j = 0; j < 3; j++)
	pjoint[j].tau_ref -= tau_w[j];
      
      armIK.P() = armFK.P();

      goto tau_transform;
    }	//------- flag_standby ---------------------------------------
    
    //---- Reference -----------------------------------------------
    switch (ctrl_mode) {
      /*      
    case MASTER_SLAVE:
      {
	BufMS buf;
	
	if( rtf_get( FIFO_SLAVE_TO_ARM0, &buf,sizeof(buf) ) == sizeof(buf) ){

	  buf.Pd.rotate(Ktl::Z, PI);
	  double insert_length = 700 - (armFK.P() - Pp).abs();
	  Ktl::Vector<3> vz = ( buf.Pd | Rs.column(0) );
	  //vz.print();
	  Ktl::Vector<3> vr = buf.Pd - vz;
	  //vr.print();
	  wref =  (Rs.column(0) * vr) / insert_length;
	  Pdref = wref * (armFK.P() - Pp) + vz;
	  //Pdref = w * ( -100*Rs.column(0) ) + vz;
	  
	  ////Pdref = wref * (armFK.P() - Pp) + vs * Rs.column(0);
	  ///Pdref =  vz;
	}
      }      
      if (!qdref3_1.solve(Jp3, Pdref)){ //IKだと暴走     qdref3算出
	rt_print("decompose %d %d \n", ret, count);
      }
      for (int j = 0; j < 3; j++)
	pjoint[j].qdref = qdref3_1[j];
      
      for (int j = 0; j < PAXIS; j++){
	Ktl::limitRange(&pjoint[j].qdref, -10.0, 10.0 );
	pjoint[j].Qdref.update(); //qddref
	Ktl::limitRange(&pjoint[j].qddref, -10.0, 10.0 );

	pjoint[j].qref += pjoint[j].qdref * dt;      //位置目標値を求める
      }
      for (int j = 0; j < 3; j++){
	armIK.q[j] = pjoint[j].qref;
      }
      armIK.q[5] = pjoint[3].qref;
      armIK.forward_kinematics();

      break;
      */
    case MODE_TASK:
      fifo_master.get( &ubuf, sizeof(ubuf) );
      
      if( ubuf.trigger == 1 )
	operation_mode = OP_HEAD;
      else if( ubuf.trigger == -1 )
	operation_mode = OP_DISABLE;

      for(int j=0;j<3;j++)
	ws[j] = exw * ws[j] + (1-exw) * ubuf.w[j] * 1e-6;
      vs = exw * vs + (1-exw ) * ubuf.v * 1e-6;

      if(operation_trigger == 1) {
	operation_trigger = 0;
	vel.create(mval, 0.0, shifting_time, time); 
	flag_stopping = false;
	flag_vtv = true;
      } else if (operation_trigger == -1) {
	operation_trigger = 0;
	vel.create(0.0, vel.get(time), shifting_time, time);
	flag_stopping = true;
	flag_vtv = true;
      }
      if( flag_stopping && !flag_vtv ){
	operation_mode = OP_DISABLE;//完全に速度0に到達後操作フラグを下げる
	flag_stopping = false;
      }


      if(operation_mode == OP_STEP_VIEW){
	switch (mindex) { //sw:scope角速度スケーリング
	case 0: ws[0] = vel.get(time);  break;
	case 1: ws[1] = vel.get(time);  break;
	case 2: ws[2] = vel.get(time);  break;
	case 3:    vs = vel.get(time); 
	  break;
	}
	//ws[mindex] = vel.get(time, &flag_vtv);
      }

      // ws vs------------------------------------
      //wref = Rs * Tgt * ws; //連続運動学
      endoscope.forward_kinematics();
      wref = Rs * ~endoscope.R() * Tgt * ws; //連続運動学   
      //wref = armFK.Rr() * Tgt * ws; //連続運動学   
      //Tgt:マスタ系からスレーブ系への座標変換行列の転置
      //wref = holder.FK.Rr() * scope.Rr() * Tr * Tgt * ws; //左右:水平
      
      Pdref = wref * (armFK.P() - Pp) + vs * Rs.column(0);
      //Pdref = wref * (armFK.Pr() - Pp) + vs * armFK.Rr().column(0);
      //アーム先端速度目標値の算出


      if(operation_mode == OP_STEP_CARTESIAN){
	switch (mindex) { //sw:scope角速度スケーリング
	case 0: Pdref[0] = vel.get(time);  break;
	case 1: Pdref[1] = vel.get(time);  break;
	case 2: Pdref[2] = vel.get(time);  break;
	  //case 3:    qdrevs = sv[1] * svel.get(time, &flag_vtv); 
	break;
	}
	//Pdref[mindex] = vel.get(time, &flag_vtv);
      }

          
      if (!qdref3_1.solve(Jp3, Pdref)){ //IKだと暴走     qdref3算出
	rt_print("decompose %d %d \n", ret, count);
      }
      for (int j = 0; j < 3; j++)
	pjoint[j].qdref = qdref3_1[j];
      if (!qdref3_2.solve( Jr3, wref-pjoint[0].qdref*armFK.s(0))){
	rt_print("decompose2 %d %d \n", ret, count);
      }
      pjoint[3].qdref = qdref3_2[2];

      if(operation_mode == OP_STEP_JOINT){
	pjoint[tj].qdref = vel.get(time);//, &flag_vtv);
      }

      for (int j = 0; j < PAXIS; j++){
	Ktl::limitRange(&pjoint[j].qdref, -10.0, 10.0 );
	pjoint[j].Qdref.update(); //qddref
	Ktl::limitRange(&pjoint[j].qddref, -10.0, 10.0 );

	pjoint[j].qref += pjoint[j].qdref * dt;      //位置目標値を求める
      }

      //armIKを目標値関節変位に合わせて更新
      for (int j = 0; j < 3; j++) 
	armIK.q(j) = pjoint[j].qref;
      armIK.q(5) = pjoint[3].qref;
      armIK.q(3) = armFK.q(3);
      armIK.q(4) = armFK.q(4);

      armFK.forward_kinematics();
      
      /*
      for (int j = 0; j < PAXIS; j++) {	//外力によるパッシブムーブメント
	qe[j] = fabs(pjoint[j].tau_th / pjoint[j].Kpp);
	if( pjoint[j].q > pjoint[j].qref + qe[j] ){
	  pjoint[j].flag_tau_ext = true;
	  pjoint[j].qref  = pjoint[j].q - qe[j];
	  pjoint[j].qdref = pjoint[j].qd;
	}
	else if( pjoint[j].q < pjoint[j].qref - qe[j] ){
	  pjoint[j].flag_tau_ext = true;
	  pjoint[j].qref = pjoint[j].q + qe[j];
	  pjoint[j].qdref = pjoint[j].qd;
	}
	else{
	  pjoint[j].flag_tau_ext = false;
	}
      }
      */
      //printf("qdref %f at %d\n",pjoint[0].qdref,count);
      break; //TASK
      
    case MODE_TEST_IK_MOTION:
      for (int j = 0; j < 3; j++)
	armIK.P()[j] = Pwave[j].generate(time);
      armIK.P() += P0;
      
      //rotation //各軸まわりの回転角で与える
      armIK.R() = Ktl::XYZMatrix(Pwave[3].generate(time),
				 Pwave[4].generate(time),
				 Pwave[5].generate(time));

      armIK.inverse_kinematics();	//逆運動学を解く
      
      for (int j = 0; j < 3; j++)
	pjoint[j].qref = armIK.q(j);
      
      for (int j = 0; j < PAXIS; j++) {
	pjoint[j].Qref.update();  //qdref
	pjoint[j].Qdref.update(); //qddref
      }
      //先端速度
      //Pdref = IK.tip_velocity(holder.qdref);
      break;
      
    case MODE_TEST_FK_MOTION:
      for (int j = 0; j < PAXIS; j++){
	pjoint[j].qref = Qwave[j].generate(time); //関節角度の目標値[rad]
	pjoint[j].Qref.update();  //qdref
	pjoint[j].Qdref.update(); //qddref
      }
      //printf("count %d\n",count);
      //先端速度
      //Pdref = IK.tip_velocity(holder.qdref);
      break;
      
    default:
      rt_print("ctrl thread : unknown command %d\n", command);
      continue;
    }	//switch(ctrl_mode)

    //======================================================================

    if (time < time_startup) { //プリプロセス中は加速度0
      double r = (time - time_startup) / pt + 1.0;
      for (int j = 0; j < PAXIS; j++){
	pjoint[j].qdref  *= r;
	pjoint[j].qddref = 0;
      }
    }


    if( flag_feedback ){
      for (int j = 0; j < PAXIS; j++) {//---------- 位置制御則 ------
	pjoint[j].limit_qref();
	pjoint[j].feedback_control(); //tau_fb を更新
      }
    }
    else if( operation_mode != OP_DISABLE ){
      if( operation_mode_pre == OP_DISABLE ){
	active_count=0;
	for (int j = 0; j < PAXIS; j++)
	  tau_fb_temp[j] = pjoint[j].tau_fb;	  
      }

      double r = (double)active_count / 100;
      if( r > 1.0 ) r = 1.0;
      
      for (int j = 0; j < PAXIS; j++) {//---------- 位置制御則 ------
	pjoint[j].limit_qref();
	pjoint[j].feedback_control(); //tau_fb を更新

	pjoint[j].tau_fb = r * pjoint[j].tau_fb + (1-r) * tau_fb_temp[j];
      }

      active_count++;
    }
    else{
      
      if( operation_mode_pre != OP_DISABLE ){
	passive_count = 0;
	for (int j = 0; j < PAXIS; j++)
	  tau_fb_temp[j] = pjoint[j].tau_fb;	  
      }

      double r = 1.0 - (double) passive_count / 2000;
      Ktl::limitRange( &r, 0.0, 1.0 );
      
      for (int j = 0; j < PAXIS; j++) {
	pjoint[j].qref = pjoint[j].q; //完全パッシブには必要


	pjoint[j].tau_fb = r*r * tau_fb_temp[j]; //ゆっくり0に戻す

	//pjoint[j].tau_fb = 0.0; //これだと動き終わりがカクつく  

      }
      armIK.P() = armFK.P(); //更新しておく
      //dref = distance;

      passive_count++;
    }

    operation_mode_pre = operation_mode;
    

    
    //----------- 動力学の計算・フィードフォワード補償 ---------------
    for (int j = 0; j < PAXIS; j++){
      pjoint[j].feedforward_control(-1,1,1);
    }
    pjoint[1].tau_ff  = pjoint[1].calcZ(0,1,1);
    //pjoint[1].tau_ff += pjoint[1].K * sin( 2.5*pjoint[1].q - 2.1*PI );
    pjoint[1].tau_ff += -pjoint[1].K * pjoint[1].q;

    pjoint[2].tau_ff  = pjoint[2].calcZ(0,1,1);
    //pjoint[2].tau_ff  = pjoint[2].calcZ(0,1,1);
    //pjoint[2].tau_ff += pjoint[2].K * sin( 1.4*pjoint[2].q + 0.17*PI );
    pjoint[2].tau_ff += pjoint[2].K * sin( 1.2*pjoint[2].q + 0.7 );

    for(int j=0;j<3;j++)
      pjoint[j].tau_ff -= tau_w[j];


    for (int j = 0; j < PAXIS; j++) {
      pjoint[j].tau_ref = 0.0;
      
      //if( flag_feedback || operation_mode != OP_DISABLE )
      pjoint[j].tau_ref += pjoint[j].tau_fb;

      if( flag_feedforward )
	pjoint[j].tau_ref += pjoint[j].tau_ff;  //動力学補償を加える
    }
    
    //----------- 外力推定処理 -------------------------------------
    for (int j = 0; j < PAXIS; j++){ 
      pjoint[j].tau_ext = ex_f * pjoint[j].tau_ext
	+ (1 - ex_f) * pjoint[j].tau_fb;
    }


  tau_transform:
    //------------- 駆動力制御 -------------------------------------
    //準備機動中の目標トルク調整
    if( time < time_startup ){
      double r = (time - time_startup) / pt + 1.0;
      for (int j = 0; j < PAXIS; j++)
	pjoint[j].tau_ref *= r;
    }

    
    for (int j = 0; j < PAXIS; j++) {
      pjoint[j].limit_tau_ref();
      pjoint[j].update_Fair_ref();
      pjoint[j].update_dPref();
      pjoint[j].dpressure_control(flag_ev_open);
      output.u[j] = pjoint[j].u; //指令電圧
    }
    
    if(stepper_trigger == 1) {
      stepper_trigger = 0;

      output.dout[2] = flag_stepping_backward;//direction

      //fpt.create(fpmax, 0.0, 1.2, time, Ktl::TRJ_ACCEL); 
      fpt.create(fpmax, 0.0, 1.2, time, Ktl::TRJ_LINEAR);//たまに脱調する
      flag_stepping = true;
      flag_stopping = false;
      flag_fpt = true;
    } else if (stepper_trigger == -1) {
      stepper_trigger = 0;
      
      fpt.create(0.0, fpt.get(time), 0.2, time,Ktl::TRJ_LINEAR);
      //fpt.create(0.0, vel.get(time), 0.2, time,Ktl::TRJ_ACCEL);
      flag_stopping2 = true;
      flag_fpt = true;
    }
    
    if( flag_stopping2 && !flag_fpt ){
      flag_stepping = false;
      flag_stopping2 = false;
    }

    if( flag_stepping ){
      fp = fpt.get(time);//, &flag_fpt);

    }
    else fp = 0.0;
    
    output.f[1] = fp;
    output.dout[0] = !flag_ev_open;
    output.dout[3] = true; //for RF box
    
    ts01.write_data( &output );

    //printf("armIK.Pr()[0]=%f,armIK.Pr()[1]=%f,armIK.Pr()[2]=%f\n",
    //		armIK.Pr()[0], armIK.Pr()[1], armIK.Pr()[2]);
    
    
    //--------- Data recording ---------------------------------------
    if (count % log_skip == 0) {
      buf.time = time; // 1
      // mnp.datに保存
      int i = 0;
      for (int j = 0; j < 3; j++) 
	buf.mnp_data[i++] = armIK.P()[j] - P0[j];// 2 3 4
      for (int j = 0; j < 3; j++) 
	buf.mnp_data[i++] = armFK.P()[j] - P0[j];// 5 6 7
      for (int j = 0; j < 3; j++)
	buf.mnp_data[i++] = Pdref[j]; // 8 9 10
      for (int j = 0; j < 3; j++)
	buf.mnp_data[i++] = Pd[j];  // 11 12 13
      for (int j = 0; j < 3; j++)
	buf.mnp_data[i++] = Pp[j]; // 14 15 16

      buf.mnp_data[i++] = distance;
      for (int j = 0; j < 3; j++)
	buf.mnp_data[i++] = n[j];
      for (int j = 0; j < 3; j++)
	buf.mnp_data[i++] = ws[j];
      buf.mnp_data[i++] = vs;
      
      // axis*.datに保存
      for (int j = 0; j < PAXIS; j++) {
	buf.axis_data[LOG_Q_REF][j] = pjoint[j].qref; //2
	buf.axis_data[LOG_Q_REF_D][j] = pjoint[j].qdref; //3
	buf.axis_data[LOG_Q_REF_DD][j] = pjoint[j].qddref; //4
	buf.axis_data[LOG_Q][j] = pjoint[j].q; //5
	buf.axis_data[LOG_Q_D][j] = pjoint[j].qd; //6
	buf.axis_data[LOG_Q_DD][j] = pjoint[j].qdd; //7
	buf.axis_data[LOG_TAU_REF][j] = pjoint[j].tau_ref; //8
	buf.axis_data[LOG_TAU][j] = pjoint[j].tau; //9
	buf.axis_data[LOG_TAU_EXT][j] = pjoint[j].tau_ext; //10
	buf.axis_data[LOG_ZFF][j] = pjoint[j].tau_ff; //11
	buf.axis_data[LOG_F_AIR_REF][j] = pjoint[j].Fair_ref; //12
	buf.axis_data[LOG_F_AIR][j] = pjoint[j].Fair; //13
	buf.axis_data[LOG_EXTRA1][j] = pjoint[j].dPe_int; //14
	buf.axis_data[LOG_EXTRA2][j] = pjoint[j].u;//15
	
      }
      
      //ユーザ空間 main() へデータを送信
      ret = fifo_log.ovrwr_put(&buf, sizeof(buf));
    }
    
    count++;
    //task.wait();
  }	//while(1)

  return 0;
}	//Controller::thread()


Controller* ctrl;

//リアルタイムタスク関数
static void *thread_ctrl(void *p_arg) {
  return ctrl->thread(p_arg);
}

/*******************************************************************
 *     FIFO handler
 *******************************************************************/

int fifo_handler(void* p_msg) {
  int command = *(int*)p_msg;
  task.send(command);
  
  return 0;
}


int main() {
  Ktl::enable_debug();
  //Ktl::ManagedProcess mp("av3_ctrl");//for process management
  Ktl::FIFOHandler handler("av3_ctrl");
  
  ctrl = new (SHM_NAME) Controller(0);

  Ktl::FileReader fr;
  fr.set_list( ctrl->parameterList() );
  
  fr.read_file("../modules/default.av3",true);
  //GUIから呼び出したときはパスが合わないの開かれない
  

  const char* ip_address = "192.168.1.100";

  Ktl::ProcessManager sim_module("av3_sim");

  if( sim_module.exists() ){
    ip_address = "127.0.0.1";
  }
  if( !ts01.open(ip_address) ){
    fprintf(stderr,"cannot open ts01 %s\n",ip_address);
    return -1;
  }

  
  //--- SSI -----------------------------------------------
  short ssi_clock  = 16;  // 16 * 100 ns
  short ssi_timeout = 40000;// / 8; // 40000ns / 8ns  //30usでは短すぎる

  for(int j=0;j<ADOF;j++){
    ts01.setup_ssi( j, ssi_clock, BIT[j]+1, ssi_timeout );
  }

  ts01.set_dout_mode(1,true);//ステッピングモータ用にクロックモードに変更
  
  //-- ctrl -------------------------------------------
  task.create(thread_ctrl, 2, (void*) 0);
  
  handler.handle(fifo_handler);

  //====== cleanup ========================================
  task.cancel(); //thread cancel //ts01より先に．
  task.join();

  ts01.stop_sampling();//この順番

  TS01OutputData output;
  for(int j=0;j<TS01_DO_CH_NUM;j++)
    output.dout[j] = false;
  
  output.dout[0] = true; //!flag_ev_open;
  
  for (int j = 0; j < PAXIS; j++)
    output.u[j] = 5.0;

  ts01.write_data( &output );    
  ts01.close();
  
  Controller::operator delete(ctrl, SHM_NAME);
  
  rt_print("control module has been removed.\n");

  return 0;
}


