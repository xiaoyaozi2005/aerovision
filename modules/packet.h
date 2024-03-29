class RBuf : public Ktl::CommunicationInterface{
public:
  int trigger;
  int w[3];
  int v;
  RBuf() : Ktl::CommunicationInterface(0){

  }
  int calc_sum()const{
    return w[0]+w[1]+w[2] + v;

  }
};

