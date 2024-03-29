#ifndef SIM_H
#define SIM_H

#include "controller.h"

class Simulator : public Ktl::RTShm{

 public:

  Arm arm;

  Ktl::Vector<3> Pp;

  Simulator() {
    //Pp.zero();
  }

  void init();
  void* thread(void*);
 private:

};

#endif
