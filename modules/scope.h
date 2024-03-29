
#ifndef _SCOPE_KINEMATICS
#define _SCOPE_KINEMATICS

class Scope : public Ktl::SerialMechanism<2>{
public:
  Scope(){
    setup_link( 0, 0.0, Ktl::Z);
    setup_axis( 0, Ktl::Z);
    setup_link( 1, 0.0, Ktl::Z);
    setup_axis( 1, Ktl::Y);  
  }
  /*
  int forward_kinematics(){
    //シリアルリンクではない
    //q[0] : 屈曲方向
    //q[1] : 屈曲角度

    s(1) = Ktl::Matrix<3,3>( s0(0), q(0) ) * s0(1);
    R() = Ktl::Matrix<3,3>( s(1) , q(1) );
    P() = R() * l0(1);

    return 0;
  }
  */
  /*
   * カメラの向きは -z方向 つまりカメラモニタにはz軸がこちら向かって見える
   */
  /*
  void draw(){
    Ktl::GL::drawCylinder( 10, -310, 10 );
    glPushMatrix();
    glTranslatef( 5.0, 0.0, -310.0);
    Ktl::GL::drawCuboid( 20.0, 5.0, 12.0, Ktl::GL::BLUE );
    glPopMatrix();

    glPushMatrix();
    //glCallList(list_link[0]);

    glRotated( q[1]*DEG , V3(s[1]) );
    //glCallList(list_link[1]);
    Ktl::GL::drawCylinder( l0[1], 10.0,  Ktl::GL::SUS);
    glTranslated( V3(l0[1]) );
    Ktl::GL::drawAxis( 0.6, 20.0 );
    glPopMatrix(); //閉じる
  }
  */
  /*
  void look(){
    Ktl::GL::rotateMatrix( ~(R*Tc) );
    glTranslated(  V3(-P) );
  }
  void draw_viewing_coordinate(){
    glPushMatrix();
    glTranslated( KTLV3( P ) );
    Ktl::GL::rotateMatrix( R*Tc );
    Ktl::GL::drawAxis( 1.5, 80.0 );
    glPopMatrix();
  }
  */
};

#endif
