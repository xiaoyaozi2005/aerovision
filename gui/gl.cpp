#include <unistd.h>
#include <ktl.h>
#include <QtWidgets>
#include <QtUiTools>
#include "../modules/controller.h"
#include "../modules/sim.h"
#include "gl.h"

extern Controller *ctrl;
extern Simulator *sim;

LGLWidget::~LGLWidget(){
}

LGLWidget::LGLWidget(){
  eye.setupView( 300.0, -1000.0, 600.0,
		 300.0,     0.0, 200.0,
		 0, 0, 1 );
  
  projection.setOrtho(800);
  
  light[0].setPosition( 0.0, 0.0, 1000.0 );
  light[0].setWhite( 0.3 );
  light[0].setIndex( GL_LIGHT1 );
  
  light[1].setPosition( 500.0, -100.0, 1200.0 );
  light[1].setWhite( 0.0 );
  light[1].setIndex( GL_LIGHT2 );

  //flag_axis = true;
}


void LGLWidget::create(){
  drawerFK.setMechanism(&ctrl->armFK);
  drawerIK.setMechanism(&ctrl->armIK);
  drawerFK.setParallelOffset(1, 39.0, 180.0/DEG);
  drawerIK.setParallelOffset(1, 39.0, 180.0/DEG);
  drawerFK.setParallelOffset(2, 32.0, 0.0/DEG);
  drawerIK.setParallelOffset(2, 32.0, 0.0/DEG);
  drawerFK.create();
  drawerIK.create();
}

void LGLWidget::draw(){

  for(int i=0;i<LIGHT_NUM;i++){
    light[i].apply();
    light[i].draw();
  }

  //if(flag_axis)
  KQt::GL::drawAxis(4.0, 300.0);
  
  glPushMatrix();
  glTranslatef( 300.0, 0.0, 0.0 );
  KQt::GL::drawGridPlane(600,50,KQt::GL::BLUE);
  glPopMatrix();
  drawerFK.draw();
  
  glPushMatrix();
  glTranslatef( KTLV3( ctrl->armFK.P() ) );
  //ctrl->armFK.P().print();
  //KQt::GL::rotateMatrix(ctrl->armFK.Tw);
  KQt::GL::rotateMatrix(ctrl->Rs );
  //アームの姿勢行列と内視鏡の姿勢行列は違う
  /*
  glRotatef( 90.0, 0,1,0 );
  glRotatef( 90.0, 0,0,1 );
  glTranslatef( 0.0, 0.0, 600.0);
  laparoscope.draw();
  */

  KQt::GL::drawCylinder( 450.0, 10.0, KQt::GL::SILVER);

  KQt::GL::drawAxis(6.0, 80.0);  
  glPopMatrix();
  
  if( ctrl->ctrl_mode != MODE_TEST_FK_MOTION )
    drawerIK.draw();


  /*
  for(int j=0;j<PAXIS;j++){
      glPushMatrix();
    if( j < 3 )
      glTranslated( KTLV3(ctrl->armFK.A[j]) );
    else
      glTranslated( KTLV3(ctrl->armFK.P() ) );

    if( ctrl->pjoint[j].flag_sat_tau ){ //トルク飽和　赤
      KQt::GL::drawCylinder(10,20,8,KQt::GL::RED);
      //glutSolidSphere(40.0, 10,10);
    }
    if( ctrl->pjoint[j].flag_tau_ext ){ // パッシブモードトルク閾値越え　青
      KQt::GL::drawCylinder(10,30,8,KQt::GL::BLUE);
      //glutSolidSphere(40.0, 10,10);
    }

    if( ctrl->pjoint[j].range_limit ){ //可動限界　黄
      KQt::GL::drawCylinder(20,40,8,KQt::GL::YELLOW);
      //glutSolidSphere(40.0, 10,10);
    }
    glPopMatrix();
  }
  */
 
  glPushMatrix();
  glTranslatef( KTLV3(ctrl->armFK.P()) );

    //KQt::GL::drawSphere( 10.0, 8, 8, KQt::GL::GREEN );
  //KQt::GL::drawSphere( 10.0, 8, 8, KQt::GL::GREEN );
  KQt::GL::drawVector( 5*ctrl->Pdref, 10.0, KQt::GL::RED );
  KQt::GL::drawVector( 5*ctrl->Pd   , 10.0, KQt::GL::GREEN );
  
  //KQt::GL::rotateMatrix( ctrl->armFK.R() );
  //KQt::GL::drawAxis(4.0, 100.0);  
  glPopMatrix();

  //estimated pivot position
  glPushMatrix();
  glTranslatef( KTLV3(ctrl->Pp) );
  KQt::GL::drawVector( 1000*ctrl->wref, 18.0, KQt::GL::RED );
  KQt::GL::drawSphere(15.0, 10,10, KQt::GL::YELLOW);
  glPopMatrix();

  //---SIM------------------------------------------
  glPushMatrix();
  glTranslatef( KTLV3(sim->Pp) );
  KQt::GL::drawSphere(18.0, 10, 10, KQt::GL::GREEN);
  glPopMatrix();

  //printf("q=%f\n",ctrl->armFK.q(0));
  //printf("q=%f\n",test->q(0));
  /*
  glPushMatrix();
  glTranslatef( KTLV3(ctrl->Pref) );
  KQt::GL::drawSphere( 30.0, 8, 8, KQt::GL::RED );
  glPopMatrix();

  glPushMatrix();
  glTranslatef( KTLV3(ctrl->armIK.P() ));
  KQt::GL::drawSphere( 35.0, 8, 8, KQt::GL::BLUE );
  glPopMatrix();
  */
  
}
