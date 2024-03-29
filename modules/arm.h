#ifndef __ARM
#define __ARM

class Gimbal : public Ktl::SerialMechanism<3>{
 public:

  Gimbal();
  void forward_kinematics(const Ktl::Vector<3>& P0=Ktl::o(),
			  const Ktl::Matrix<3,3>& R0=Ktl::I());
  //void draw(bool flag_continue=false)const;
};


#define ADOF 6
class Arm : public Ktl::SerialMechanism<ADOF>{
 public:
  double m[ADOF]; //[Mg]
  double Ig[ADOF]; //[g m^2]

  //Ktl::Matrix<3, 3> Tw;//(Ktl::Y, 45.0 / DEG); //ジンバルオフセット角
  
 private:
  //Ktl::Vector<3> lo;//arm手先からPtまでのベクトル
  //Ktl::Vector<3> Pt; //gimbalの回転中心位置

 public:
  Arm();
  //Ktl::Matrix<3,3> Jp3()const; // pose
  //Ktl::Matrix<3,3> Jr3()const; // pose
  /*
  Ktl::Vector<3> Pr()const{return Pt;} // position
  Ktl::Vector<3>& Pr(){return Pt;} // position

  void forward_kinematics(const Ktl::Vector<3>& P0=Ktl::o(),
			  const Ktl::Matrix<3,3>& R0=Ktl::I());
  void inverse_kinematics(const Ktl::Vector<3>& P0=Ktl::o(),
			  const Ktl::Matrix<3,3>& R0=Ktl::I());
  Ktl::Matrix<3,ADOF> Jp()const; // pose
  Ktl::Matrix<3,ADOF> Jr()const; // pose


  Ktl::Vector<ADOF> inverse_dynamics( const Ktl::Vector<ADOF>& qdd,
				      const Ktl::Vector<ADOF>& qd);
  void draw(bool flag_continue=false)const;
  */
};





#endif
