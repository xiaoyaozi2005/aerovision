#ifndef __CONTROLLER
#define __CONTROLLER

#include "arm.h"
#include "scope.h"

#define PAXIS 4
#define SHM_NAME "av3m"
static const int DOF = 6;
#define BUF_TIME_SIZE 50

#define TG0 Ktl::Matrix<3,3>(Ktl::Z,0.5*M_PI)

enum ControlMode{
  //MASTER_SLAVE,
  MODE_TASK,
  MODE_TEST_IK_MOTION,
  MODE_TEST_FK_MOTION,
};



enum LOG_DATA{  
  LOG_Q_REF, // 2  ( file column )
  LOG_Q_REF_D, 
  LOG_Q_REF_DD, 
  LOG_Q,      // この
  LOG_Q_D,    // 3つは
  LOG_Q_DD,   // 必ずこの順番で (学習のため)
  LOG_TAU_REF,
  LOG_TAU,

  LOG_TAU_EXT,
  LOG_ZFF, 
  //LOG_Z,

  LOG_F_AIR_REF,
  LOG_F_AIR,
  
  LOG_U_SAT,
  LOG_EXTRA1,
  LOG_EXTRA2,

  LOG_LAST
};

#define AXIS_LOG_SIZE LOG_LAST
#define MNP_LOG_SIZE 24

typedef struct{
  float time;
  float mnp_data[MNP_LOG_SIZE];
  float axis_data[AXIS_LOG_SIZE][DOF];
} LogBuf;

#define POSITIONING_MODULE_DOWN    0x01
#define AIR_MODULE_DOWN            0x02
#define EXCESSIVE_VELOCITY         0x10


enum OperationMode{
  OP_DISABLE,
  OP_STEP_JOINT,
  OP_STEP_CARTESIAN,
  OP_STEP_VIEW,
  OP_HEAD,
  /*
  OP_HEAD_AROUND, //(上下左右)
  OP_HEAD_ZOOM,
  OP_HEAD_ROLL,
  OP_HEAD_ALL, 
  */
};


class Controller : public Ktl::RTShm{
 protected:
  const int id;
  double dt;

 public:
  int period;
  int ctrl_mode;
  bool flag_feedforward;
  bool flag_feedback;

  Ktl::Wave Pwave[6];
  Ktl::Wave Qwave[PAXIS];
  Ktl::Wave uwave[PAXIS];
  double fc_Fext;	//カットオフ周波数
  double Ms; //質量 内視鏡 [Mg]
  double Mh; //質量 holder 可動部 [Mg]
  double Ig[PAXIS];
  double m[PAXIS];

  int state;

  int log_skip;

  //---受け渡し用
  double fc_u;
  double fc_qd;
  double fc_qdd;
  
  double time;
  bool emergency_flag;// = 0;

  bool flag_enc; //エンコーダが初期化されたかどうか
  bool flag_init_pulse;
  bool flag_initializing_enc;

  bool flag_ptp;
  int flag_error; //何か一つでも問題があればtrue

  bool flag_scope_mount;
  double dM;//スコープ質量の増分値（オンライン調整）

  Scope endoscope;
  
  //-- Arm --------------------------
  //スレーブ系からマスタ系への座標変換 = TG0 マクロ定義
  //さらにTG0をZ軸まわりにtgだけ回転させ，オフセット角を作る
  Ktl::Matrix<3,3> Tg;
  //マスタ系からスレーブ系への座標変換行列
  // ~は転置を表すが，直交行列の場合はこれが逆行列に等しい
  Ktl::Matrix<3,3> Tgt; // = ~Tg;

  Arm armIK;
  Arm armFK;
  Ktl::Matrix<3,3> Rs;	//内視鏡姿勢行列
  Ktl::Matrix<3,3> Rref;	//先端目標姿勢行列
  Ktl::Vector<3> Pref;		//先端目標位置ベクトル
  Ktl::Vector<3> Pdref;	//先端速度の目標値
  Ktl::Vector<3> wref;	//先端角速度の目標値
  Ktl::Vector<3> Pd;	//先端速度
  Ktl::Vector<3> w;		//先端角速度

  Ktl::Vector<3> fext;	//先端に加わる並進外力
  Ktl::Vector<3> mext;	//先端に加わるモーメント外力

  Ktl::Vector<3> P0;

  //--- pivot ------------------------
  bool flag_pivot_lock;
  double distance;
  double distance2;
  Ktl::Vector<3> Pp; //ピボット位置

  //------ joint control ---------------------------
  double ex_f; //ローパス時定数（外力）

  bool flag_limit; //可動域飽和フラグ

  bool flag_ev_open; //
  double P1[PAXIS];
  double P2[PAXIS];

  Ktl::PneumaticJoint pjoint[PAXIS];
  double Z[PAXIS];
  double Znn[PAXIS];

  double qe[4];
  //Ktl::Vector<3> tau_th;

  //--- references ------------------
  int operation_mode;
  int operation_trigger;

  //Ktl::Vector<3> wv;// view control on the panel
  Ktl::Vector<3> ws;// scope 
  double vs;

  int tj;
  int mindex;
  double mval;
  Ktl::PPTrajectory<double> vel;

  //double sv[3]; //scope並進速度スケーリング
  //double sw[3]; //scope角速度スケーリング
  int stepper_trigger;
  bool flag_stepping_backward;

  
  //コンストラクタ
 Controller(int ID): id(ID),
    ctrl_mode(MODE_TASK),
    log_skip(1), 
    dM(0.0)
  {  
    Tg  = TG0;
    Tgt = ~Tg;
    /*
    for(int j=0;j<3;j++){
      sv[j] = 1.0;
      sw[j] = 1.0;
    }
    */
  }
  
  void* thread(void*);
  void set_Pwave();

  Ktl::TaggedDataList parameterList(){
    Ktl::TaggedDataList list;
    
    list.push_data( "period",&period );
    list.push_data( "feedforward", &flag_feedforward);
    list.push_data( "mode" , &ctrl_mode );
    
    list.push_data("fc_u"   ,&fc_u);
    list.push_data("fc_qd"  ,&fc_qd);
    list.push_data("fc_qdd" ,&fc_qdd);
    list.push_data("Mh"  ,&Mh); //
    list.push_data("Ms"  ,&Ms); //mass of scope
    
    for(int j=0;j<6;j++){
      list.push_data("Pref",j,&Pwave[j]);
    }
    
    for(int j=0;j<PAXIS;j++){
      list.push_data("Qref",j,&Qwave[j]);
      list.push_data("uref",j,&uwave[j]);
    }
    
    for(int j=0;j<PAXIS;j++){
      list.push_data("Kpp", j ,&pjoint[j].Kpp);
      list.push_data("Kpd", j ,&pjoint[j].Kpd);
      list.push_data("Kpdd",j ,&pjoint[j].Kpdd);
      list.push_data("Kap", j ,&pjoint[j].Kap);
      list.push_data("Kai", j ,&pjoint[j].Kai);
      list.push_data("Kad", j ,&pjoint[j].Kad);
      
      list.push_data("M",    j ,&pjoint[j].M);
      list.push_data("C",    j ,&pjoint[j].fr.C);
      list.push_data("Fd",   j ,&pjoint[j].fr.Fd);
      list.push_data("vlim", j ,&pjoint[j].fr.vlimit);
      list.push_data("K",    j ,&pjoint[j].K);
      list.push_data("Zo",   j ,&pjoint[j].Zo);
    }

    //printf("parameterList done\n");
    return list;
  }
};



#endif
