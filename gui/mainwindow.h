#ifndef LMAINWINDOW_H
#define LMAINWINDOW_H

#include <kqt.h>
#include "../modules/controller.h"
#include "gl.h"

class WeightAdjuster : public KQt::ValueChanger{
  Q_OBJECT
public:
  WeightAdjuster();
public slots:
  void updateDisplay();
  void setMount(bool flag);
private:
  QLCDNumber* weightLabel;
  QTimer* ptimer;
  
private slots:
  void operateValue(int id, int trigger, int direction);

};

class MainWindow : public KQt::MainWindow, public KQt::LogWriter
{
Q_OBJECT

 public:
  MainWindow();
  ~MainWindow();
  
  QDialog* parameterDialog;
    
  KQt::ParameterManager pm;
  //KQt::BufferedParameterManager pm;
  Ktl::FileReader fr;

  KQt::StartStopManager* startstop;  
  KQt::ModeSelector* modeSelector;
  KQt::DataLogger* dataLogger;
  KQt::Plotter* plotter;
  KQt::SignalMonitor* monitor;
  KQt::ReferenceDialog* referenceDialog;

  WeightAdjuster* weightAdjuster;
  LGLWidget* glwidget;
  QLabel* stateLabel;
  KQt::ToggleLabel* evLabel;
  
  Ktl::ModuleHandler ctrl_module;
  Ktl::ModuleHandler sim_module;

  void getParam();
  
  //-- override from MainWindow ------------
  void loadFile(const QString &fileName);
  bool saveFile(const QString &fileName);

  private slots:
  bool start();
  void stop();
  void periodic();  
  void execParameterDialog();
  void execReferenceDialog();
  void applyReferences();
  void sendCommand(int id);
  void operateJointMotion(int id,int trigger, int direction);
  void operateCartesianMotion(int id,int trigger, int direction);
  void operateViewMotion(int id,int trigger, int direction);
  void operateStageMotion(int id,int trigger, int direction);
  void updateWidgets();//
  void operateScopeMass(int id, int trigger, int direction);
  //------------------------------------------

  //-- override form LogWriter -----------------
  virtual bool write_logfile(void* data,size_t size) override;
  virtual int open_logfile() override;
  virtual void close_logfile() override; 
  //----------------------------------------------

 private:
   void layoutWidgets();

   FILE *MNP_LOG_FILE;
   FILE *AXIS_LOG_FILE[DOF];  
   char data_filename[128];
   char axis_data_filename[DOF][128];  
   char logdir[32];//="log-latest";
   
};


#endif
