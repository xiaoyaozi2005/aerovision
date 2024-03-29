#include<ktl.h>
#include"arm.h"

Arm::Arm()  :
   Tw(Ktl::Y, 45.0 / DEG) //ジンバルオフセット角
{
  /*
  setup_link(0, 52, Ktl::Z);  //setup_link(関節番号、リンク長さ、初期位置の方向)
   //lo.set( 27+65.4*sqrt(2), 0.0, 32+52.4*sqrt(2) );
   */
  //---------------------------
   //set( 52, Ktl::Z, Ktl::Z, 0, Ktl::Vector<3>(7.0, 0.0, 0.0) );
   set( 7.0, Ktl::Z, Ktl::Z, Ktl::ROTARY, Ktl::Vector<3>(14.0, 0.0, 39.0) );
  
   set( 345, Ktl::Z, Ktl::Y,Ktl::PARALLEL, 
       //Ktl::Vector<3>(17.0/sqrt(2), 0.0, 17/sqrt(2)+32.0) );
       Ktl::Vector<3>(12.02, 0.0, 44.2) );

   set( 280, Ktl::X, Ktl::Y,Ktl::PARALLEL,
       //Ktl::Vector<3>(13.0/sqrt(2)+20.0, 0.0, -13/sqrt(2) ) );
       Ktl::Vector<3>(27.07, 0.0, -7.07 ) );

   set( Tw*Ktl::Vector<3>(0.0, 0.0, 102.0 ),
	Tw*Ktl::Vector<3>(0.0, 0.0, 1.0)  );

   set( Ktl::Vector<3>(0.0, 0.0, 0.0),  
	Tw*Ktl::Vector<3>(0.0, 1.0, 0.0) );
   
   set( Ktl::Vector<3>(0.0, 0.0, 0.0), 
	Tw*Ktl::Vector<3>(1.0, 0.0, 0.0) );
   
   init();
   forward_kinematics();
 }
