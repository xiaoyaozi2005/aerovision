#include <unistd.h>
#include <ktl.h>
#include <QtUiTools>
#include "../modules/def.h"
#include "../modules/controller.h"
#include "../modules/sim.h"
#include "mainwindow.h"

Controller *ctrl;
Simulator *sim;

WeightAdjuster::WeightAdjuster(){
  QVBoxLayout* layout = new QVBoxLayout;
  setLayout(layout);
  setTitle("Scope Weiget [g]");
  //weightLabel = new QLabel("0.0");
  weightLabel = new QLCDNumber(3);
  //weightLabel->setSuffix("kg");
  QPushButton* increase_button = new QPushButton;
  QPushButton* decrease_button = new QPushButton;

  layout->addWidget(weightLabel);
  layout->addWidget(increase_button);
  layout->addWidget(decrease_button);
  
  increase_button->setIcon(QIcon::fromTheme("go-up"));
  decrease_button->setIcon(QIcon::fromTheme("go-down"));
  
  increase_button->setIconSize(QSize(50,50));
  decrease_button->setIconSize(QSize(50,50));
  addButtons( increase_button, decrease_button);

  ptimer = new QTimer( this );
  connect( ptimer, SIGNAL(timeout()), this, SLOT( updateDisplay() ));
  
  connect( this, SIGNAL(stateChanged(int,int,int)),
	   this, SLOT(operateValue(int, int, int) ));
  
  
  QPushButton* mountButton = new QPushButton("mount");
  mountButton->setCheckable(true);
  layout->addWidget(mountButton);
  connect( mountButton, SIGNAL(toggled(bool)),
	   this, SLOT(setMount(bool)));
  
  updateDisplay();
}

void WeightAdjuster::setMount(bool flag){
  ctrl->flag_scope_mount = flag;
}

void WeightAdjuster::operateValue(int id,int trigger, int d){
  //printf("WeightAdjuster %d %d %d\n",id,trigger,d);
  if( trigger > 0 ){
    ptimer->start(40);
  }
  else{
    ptimer->stop();
  }
}

void WeightAdjuster::updateDisplay(){
  weightLabel->display(ctrl->Ms*1e3);
}


void MainWindow::operateScopeMass(int id,int trigger, int d){
  //printf("WeightAdjuster %d %d %d\n",id,trigger,d);
  if( trigger > 0 ){
    ctrl->dM = d * 0.02e-3;
  }
  else{
    ctrl->dM = 0.0;
  }
}


MainWindow::~MainWindow(){
  if( ctrl_module.exists() ){
    ctrl_module.close_handle();
    ctrl_module.unload();
  }
  Controller::operator delete( ctrl, SHM_NAME );
  Simulator::operator delete( sim, "sshm" );
}

MainWindow::MainWindow(){
  
  ctrl_module.rgst( "av3_ctrl", "../modules/ctrl_module");
  
  if( ctrl_module.exists() ){
    qDebug() << "Error: ctrl_module seems running.\n"
	     << "This is forbidened so far.\n"
	     << "Try after ctrl_module is removed.\n";
    exit(EXIT_FAILURE);
    //QCoreApplication::quit();//終了できない
  }

  sim_module.rgst( "av3_sim", "../modules/sim_module");
  if( sim_module.exists() ) sim_module.open_handle();

  ctrl = new (SHM_NAME) Controller(0);
  sim  = new ("sshm") Simulator();
  //moduleでのコンストラクタを優先するため先に生成
  
  setTitle(tr("aerovision3"));
  setFileExtension("av3");
  
  QWidget* centralWidget = new QWidget;
  setCentralWidget(centralWidget);

  QHBoxLayout* main_hlayout = new QHBoxLayout;
  QVBoxLayout* main_vlayout = new QVBoxLayout;
  QHBoxLayout* upper_hlayout = new QHBoxLayout;
  QHBoxLayout* lower_hlayout = new QHBoxLayout;

  startstop = new KQt::StartStopManager;
  monitor = new KQt::SignalMonitor;
  plotter = new KQt::Plotter;
  dataLogger = new KQt::DataLogger;
  referenceDialog = new KQt::ReferenceDialog(this);//pmのために親を指定
  modeSelector = new KQt::ModeSelector;
  
  stateLabel = new QLabel("unloaded");
  evLabel = new KQt::ToggleLabel;

  statusBar()->addPermanentWidget(stateLabel);
  statusBar()->addPermanentWidget(evLabel);
  statusBar()->addPermanentWidget(startstop->timeDisplay);
 
  
  QPushButton* paramButton = new QPushButton("Parameter");
  connect(paramButton, SIGNAL(clicked()),
	  this, SLOT(execParameterDialog()) );

  QPushButton* refButton = new QPushButton("Reference");
  connect(refButton, SIGNAL(clicked()),
	  this, SLOT(execReferenceDialog()) );
  connect(referenceDialog, SIGNAL(accepted()),
	  this, SLOT(applyReferences()) );

  QGridLayout* gridLayout = new QGridLayout;
  gridLayout->addWidget( paramButton, 0,1 );
  gridLayout->addWidget( refButton, 1,1 );

  startstop->setStartStopAction(this,
				&MainWindow::start, &MainWindow::stop);
  //startstop->setPeriodicAction(this, SLOT(periodic()) );
  QTimer* ptimer = new QTimer( this );
  connect( ptimer, SIGNAL(timeout()), this, SLOT( periodic() ));
  ptimer->start( 100 );
  
  glwidget = new LGLWidget;
  glwidget->start();

  QTabWidget* tab = new QTabWidget;

  //------------- Module Seting ---------------------------------

  KQt::ModuleButton* ctrlButton = new KQt::ModuleButton("ctrl",&ctrl_module);
  ctrlButton->update();

  KQt::ModuleButton* simButton = new KQt::ModuleButton("sim",&sim_module);
  simButton->update();

  QHBoxLayout* module_layout = new QHBoxLayout;

  module_layout->addWidget(ctrlButton);
  module_layout->addWidget(simButton);
  QGroupBox* module_box = new QGroupBox("Modules");
  module_box->setLayout(module_layout);
  module_box->setStyle( QStyleFactory::create("gtk"));//QGroupBox frame
  gridLayout->addWidget(module_box,3,1);
  //----------------------------------------------------------------

  centralWidget->setLayout(main_vlayout);
  main_vlayout->addLayout( upper_hlayout , 1 );
  main_vlayout->addLayout( lower_hlayout );
  setContentsMargins(0,0,0,0); //0にする
  centralWidget->setContentsMargins(0,0,0,0); //0にする
  main_hlayout->setContentsMargins(0,0,0,0); //0にする
  upper_hlayout->setContentsMargins(0,0,0,0); //0にする
  upper_hlayout->setSpacing(0); //0にする
  
  upper_hlayout->addWidget(monitor);    
  upper_hlayout->addWidget(glwidget);
  lower_hlayout->addWidget(tab);

  //---- Command Buttons ----------------------------------
  QGroupBox* commandBox = new QGroupBox("Command");
  QVBoxLayout* commandLayout = new QVBoxLayout;
  commandBox->setLayout(commandLayout);
  QButtonGroup* buttonGroup = new QButtonGroup;
  //buttonGroup->setExclusive(false);

  for(int i=0;i<COMMAND_LAST-1;i++){
    if( i== COMMAND_START || i== COMMAND_STOP ) continue;
    QPushButton* button = new QPushButton(command_text[i]);
    button->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
    buttonGroup->addButton(button,i);
    commandLayout->addWidget(button,1);
  }

  connect(buttonGroup, SIGNAL(buttonClicked(int)),
	  this, SLOT(sendCommand(int)) );

  //lower_hlayout->addLayout(gridLayout);ここがいい？
  lower_hlayout->addWidget(startstop);

  
  //--- Developer -------------------------------------------------
  QWidget* tab1 = new QWidget;
  tab->addTab(tab1,"config");
  tab1->setLayout(main_hlayout);
  main_hlayout->addWidget(plotter);    
  main_hlayout->addWidget(dataLogger);    
  main_hlayout->addLayout(gridLayout);

  paramButton->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );  
  gridLayout->addWidget( paramButton, 0,1 );
  refButton->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );  
  gridLayout->addWidget( refButton, 1,1 );
  
  main_hlayout->addWidget(modeSelector);
  
  main_hlayout->addWidget(commandBox);

  //--- Operator ------------------------------------------------
  QWidget* tab2 = new QWidget;
  tab->addTab(tab2,"operation");

  QHBoxLayout* layout = new QHBoxLayout;
  tab2->setLayout(layout);

  KQt::JointOperator* jointOperation = new KQt::JointOperator(4);
  connect(jointOperation, SIGNAL(stateChanged(int,int,int)),
	  this, SLOT(operateJointMotion(int,int,int)));
  layout->addWidget(jointOperation);
  
  KQt::CartesianOperator* cartesianOperation = new KQt::CartesianOperator;
  connect(cartesianOperation, SIGNAL(stateChanged(int,int,int)),
	  this, SLOT(operateCartesianMotion(int,int,int)));
  layout->addWidget(cartesianOperation);
  
  KQt::CartesianOperator* viewOperation = new KQt::CartesianOperator;
  viewOperation->setTitle("view");
  connect( viewOperation, SIGNAL(stateChanged(int,int,int)),
	  this, SLOT(operateViewMotion(int,int,int)));
  layout->addWidget(viewOperation);
  
  KQt::JointOperator* stageOperation = new KQt::JointOperator(1);
  connect( stageOperation, SIGNAL(stateChanged(int,int,int)),
	  this, SLOT(operateStageMotion(int,int,int)));
  layout->addWidget(stageOperation);

  weightAdjuster = new WeightAdjuster;
  layout->addWidget(weightAdjuster);
  connect( weightAdjuster, SIGNAL(stateChanged(int,int,int)),
	   this, SLOT(operateScopeMass(int, int, int) ));
  //tab->addTab(jointOperation,"operation");
  //lower_hlayout->addStretch();
  //qApp->setStyle( new QPlastiqueStyle );

  //qApp->setStyle( new QGtkStyle );
  
  //--------- Setting Monitor --------------------------------------
  monitor->setAxes("1","2","3","4",NULL);
#define DATA(name) (&ctrl->name)

  monitor->addType("q");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData( -60.0, 60.0, 0, DEG,
			   DATA(pjoint[j].qref), DATA(pjoint[j].q));

  monitor->addType("qd");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData(-100.0, 100.0, 0,  DEG,
			  DATA(pjoint[j].qdref),DATA(pjoint[j].qd)) ;
			  
  monitor->addType("qdd");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData(-100.0, 100.0, 0, DEG,
			  DATA(pjoint[j].qddref),DATA(pjoint[j].qdd));
  
  monitor->addType("tau");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData( -500.0, 500.0, 0, 1,
			   DATA(pjoint[j].tau_ref),DATA(pjoint[j].tau),
  			   DATA(pjoint[j].tau_ff), DATA(pjoint[j].tau_fb));

  monitor->addType("tau_ext");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData( -100.0, 100.0, 0, 1,
			   DATA(pjoint[j].tau_ext), DATA(pjoint[j].tau_fb));
  
  monitor->addType("Fair");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData( -100.0, 100.0, 0, 1,
			   DATA(pjoint[j].Fair_ref), DATA(pjoint[j].Fair));
  
  monitor->addType("d dP/dt");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData( -1000.0, 1000.0, 0, 1,
			   DATA(pjoint[j].dPdref), DATA(pjoint[j].dPd));
  
  monitor->addType("dPe_int");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData( -20.0, 20.0, 0, 1,
			   DATA(pjoint[j].dPe_int));
  
  monitor->addType("P");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData( 0.0, 500.0, -1, 1,
			   DATA(P1[j]), DATA(P2[j]));
  
  monitor->addType("u");
  for(int j=0;j<PAXIS;j++)
    monitor->registerData( 2.0, 8.0, 0, 1, DATA(pjoint[j].u) );

  
  monitor->addType("Px,Py,Pz FK");
  monitor->registerData( -500.0, 500.0, 0, 1, 
			  DATA(armFK.P()[0]), DATA(armFK.P()[1]), DATA(armFK.P()[2]) );

  monitor->addType("Px,Py,Pz IK");
  monitor->registerData( -500.0, 500.0, 0, 1,
			  DATA(armIK.P()[0]), DATA(armIK.P()[1]), DATA(armIK.P()[2]) );
    /*
  monitor->addType("Pdref");
  monitor->registerData( -100.0, 100.0, 0, 1,
			 DATA(Pdref[0]),DATA(Pdref[1]),DATA(Pdref[2]));
  */
  /*
  monitor.add_type("ws");
  monitor.register_data( -1.0, 1.0, 0, 1,
			 DATA(ws[0]),DATA(ws[1]),DATA(ws[2]) );
			 
  monitor.add_type("wref");
  monitor.register_data( -1.0, 1.0, 0, 1,
			 DATA(wref[0]),DATA(wref[1]),DATA(wref[2]) );
  */
  monitor->addType("gimbal");
  monitor->registerData( -80.0, 80.0, 0, DEG, 
			 DATA(armFK.q(3)),DATA(armFK.q(4)) );

  monitor->addType("Pp");
  monitor->registerData( -100.0, 100.0, 0, 1, 
			 DATA(Pp[0]),DATA(Pp[1]),DATA(Pp[2]) );
			
  monitor->addType("distance");
  monitor->registerData( 300.0, 600.0, 0, 1.0,
			DATA(distance), DATA(distance2) );

  monitor->addType("Pd");
  monitor->registerData( -100.0, 100.0, 0, 1, 
			 DATA(Pd[0]), DATA(Pd[1]), DATA(Pd[2]) );
			 
  monitor->addType("w");
  monitor->registerData( -1.0, 1.0, 0, 1,
			 DATA(w[0]),DATA(w[1]),DATA(w[2]) );
  /*
  monitor->addType("fext");
  monitor->registerData( -100.0, 100.0, 0, 1, 
			 DATA(fext[0]),DATA(fext[0]),DATA(fext[0]));
  */
  sprintf(logdir,"log-latest"); 
  sprintf(data_filename,"mnp.dat"); 

  for(int dof=0;dof<DOF;dof++)
    sprintf(axis_data_filename[dof],"axis%d.dat",dof); 
  

  //--- Logger --------------------------------------
  dataLogger->rgstReceiver( "av3_log", sizeof(LogBuf)*BUF_TIME_SIZE, this);
  dataLogger->setLogName(logdir);

  plotter->setDir( logdir );


  for(int j=0;j<DOF;j++)
    plotter->addFile(axis_data_filename[j]);
  
  plotter->addButton("q","[rad]");
  plotter->addLine(LOG_Q_REF+2,"reference");
  plotter->addLine(LOG_Q+2,"position");
  plotter->addButton("qd","[rad/s]");
  plotter->addLine(LOG_Q_REF_D+2,"qdref");
  plotter->addLine(LOG_Q_D+2,"qd");

  plotter->addButton("qdd","[rad/s^2]");
  plotter->addLine(LOG_Q_REF_DD+2,"ref");
  plotter->addLine(LOG_Q_DD+2,"qdd");

  plotter->addButton("tau","[mNm]");
  plotter->addLine(LOG_TAU_REF+2,"ref");
  plotter->addLine(LOG_TAU+2,    "Fdr");
  plotter->addLine(LOG_ZFF+2,"Zff");

  plotter->addButton("Fair","[N]");
  plotter->addLine(LOG_F_AIR_REF+2,"reference");
  plotter->addLine(LOG_F_AIR+2,"Fair");

  /*
  plotter->addButton("P","[kPa]");
  plotter->addLine(LOG_PRESSURE1+2,"P1");
  plotter->addLine(LOG_PRESSURE2+2,"P2");
  plotter->addButton("u","[V]");
  plotter->addLine(LOG_U1+2,"u");
  */
  plotter->incrementGroup();
  plotter->addFile(data_filename);
  const char* axis[]={"x","y","z"};
  plotter->addButton("Pr","[mm]");
  for(int j=0;j<3;j++)
    plotter->addLine(j+2,axis[j]);
  for(int j=0;j<3;j++)
    plotter->addLine(j+5,axis[j]);
  plotter->addButton("Pd","[mm]");
  for(int j=0;j<3;j++)
    plotter->addLine(j+8,axis[j]);
  for(int j=0;j<3;j++)
    plotter->addLine(j+11,axis[j]);

  plotter->addButton("fext","[N]");
  for(int j=0;j<3;j++)
    plotter->addLine(j+14,axis[j]);
  plotter->addButton("mext","[mNm]");
  for(int j=0;j<3;j++)
    plotter->addLine(j+17,axis[j]);

  modeSelector->addMode("Task",MODE_TASK);     
  modeSelector->addMode("IK",MODE_TEST_IK_MOTION);
  modeSelector->addMode("FK",MODE_TEST_FK_MOTION);
  modeSelector->setMode(2);

  QFile file("parameter.ui");
  
  QUiLoader loader;   //qdesigner を利用
  parameterDialog = (QDialog*) loader.load(&file, this);

  referenceDialog->appendWaveList("Pref",6);
  referenceDialog->appendWaveList("Qref",PAXIS);
  referenceDialog->appendWaveList("uref",PAXIS);

  KQt::JointControlEditor* jointControlEditor
    = new KQt::JointControlEditor(true); //pneumatic 
  for(int j=0;j<PAXIS;j++){
    jointControlEditor->appendJoint(KQt::UNIT_NM_RAD);
  }
  QVBoxLayout* vl = parameterDialog->findChild<QVBoxLayout*>("verticalLayout");
  vl->insertWidget(0, jointControlEditor);
  
  pm.setList( ctrl->parameterList() ); //関連付けを取得
  
  //GUI表示のスケーリングを設定・ファイルには適用されない
  for(int j=0;j<PAXIS;j++){
    pm.setWidgetMagnification("Kpp" , j , 1e-3); // mNm/rad -> Nm/rad
    pm.setWidgetMagnification("Kpd" , j , 1e-3);
    pm.setWidgetMagnification("Kpdd" ,j , 1e-3);
    pm.setWidgetMagnification("M"    ,j , 1e-3);
    pm.setWidgetMagnification("C"    ,j , 1e-3);
    pm.setWidgetMagnification("Fd"   ,j , 1e-3);
    pm.setWidgetMagnification("Zo"   ,j , 1e-3);
    pm.setWidgetMagnification("Kap" , j , 1e3 ); // V/kPa -> V/MPa
    pm.setWidgetMagnification("Kai" , j , 1e3 );
    pm.setWidgetMagnification("Kad" , j , 1e3 );
    pm.setWidgetMagnification("Qref", j , DEG); // rad -> deg
  }
  for(int j=3;j<6;j++)
    pm.setWidgetMagnification("Pref", j , DEG); // rad -> deg

  pm.excludeWidgetAssociation("mode"); //関連付け対象から外す
  pm.excludeWidgetAssociation("Ms");
  //登録されている変数アドレスをその名前に対応するwidgetに関連付ける
  pm.associateWidgets(this);//全ての対象ウィジットを包含する親を指定する

  pm.setupFileReading(fr); //frに伝達
  
  readSettings();

  if( !currentFile().isEmpty() )
    loadFile(currentFile());//

  weightAdjuster->updateDisplay();
}

void MainWindow::layoutWidgets(){
 
}

void MainWindow::operateStageMotion(int id, int t, int d){
  //printf("StageMotion : id = %d, %2d, direction = %d\n",id,t,d);
  ctrl->flag_stepping_backward = d > 0 ? true : false;
  ctrl->stepper_trigger = t;
}

void MainWindow::operateJointMotion(int id,int t,int d){
  //printf("JointMotion : id = %d, %2d, direction = %d\n",id,t,d);
  ctrl->operation_mode = OP_STEP_JOINT;

  static const double v = 0.3;

  ctrl->tj   = id;
  ctrl->mval = d*v;
  if( id == 3 )
    ctrl->mval = d*1.0;
  ctrl->operation_trigger = t;
  
}


void MainWindow::operateCartesianMotion(int id,int t,int d){
  //printf("CartesianMotion : id = %d, %2d, direction = %d\n",id,t,d);  
  ctrl->operation_mode = OP_STEP_CARTESIAN;
  const double speed = 100.0;

  ctrl->mindex=id;
  ctrl->mval = d * speed;
  if( id == 3 )
    ctrl->mval = d * 10.0;
  ctrl->operation_trigger = t;
}


void MainWindow::operateViewMotion(int id,int t,int d){
  //printf("ViewMotion : id = %d, %2d, direction = %d\n",id,t,d);
  ctrl->operation_mode = OP_STEP_VIEW;

  //GUI上では　左右、上下、前後、回転の順
  int    index[4] = { 2, 0, 3, 1};//index map,2:pan, 0:tilt, 3:zoom, 1:roll
  double speed[4] = {-0.18, 0.18, 55.0, 0.5};

  ctrl->mindex = index[id];
  ctrl->mval = d * speed[id];

  ctrl->operation_trigger = t;
}


void MainWindow::sendCommand(int cmd){
  if( !ctrl_module.exists() ){
    //show_message(" control module is not loaded ");
    printf("CtrlKit:: control module is not loaded\n ");
    return;
  }

  ctrl_module.send_command(cmd);
  statusBar()->showMessage(tr(command_text[cmd]), 2000);
  printf("sent a command of %s\n",command_text[cmd]);
}

void MainWindow::execParameterDialog(){
  pm.setValuesToWidgets(parameterDialog);
  int ret = parameterDialog->exec();

  if( ret == 1 ){
    pm.getValuesFromWidgets(parameterDialog);
  }
}

void MainWindow::execReferenceDialog(){
  pm.setValuesToWidgets(referenceDialog);
  referenceDialog->show();
}

void MainWindow::applyReferences(){
  pm.getValuesFromWidgets(referenceDialog);
}

void MainWindow::getParam(){ //GUI値を読んで変数に格納する
  pm.setValue("mode", modeSelector->currentMode() );
  pm.getValuesFromWidgets();//this);
}


bool MainWindow::start(){

  if( ctrl->state != STATE_STANDBY ){
    printf("state is not standby\n");
    return false;
  }

  getParam(); //GUIの値をバッファに取り込む
  //pm.applyBufferedValues(); //バッファの値をctrlに反映

  weightAdjuster->updateDisplay();
  
  statusBar()->showMessage(tr("running control now"), 2000);

  sendCommand(COMMAND_START);

  if( dataLogger->isSyncChecked() )
    dataLogger->reserveStart();

  //if( !flag_manual_monitor )
  if( monitor->isAvailable() ) 
    monitor->start();

  return true;
}

/*******************************************************
 *        stop button
 *******************************************************/
void MainWindow::stop(){
  if( dataLogger->isSyncChecked() )
    dataLogger->stop();

  sendCommand(COMMAND_STOP);

  statusBar()->showMessage(tr(""), 2000);
}


void MainWindow::periodic(){
  stateLabel->setText(state_text[ctrl->state]);
  evLabel->set(ctrl->flag_ev_open);
}

void MainWindow::updateWidgets(){
  pm.setValuesToWidgets();//this);
  modeSelector->setMode( pm.getValueAsInt("mode") );
  weightAdjuster->updateDisplay();
  //referenceDialog->updateDisplay();
}

void MainWindow::loadFile(const QString &fileName){
  fr.read_file(fileName.toStdString() ,true);

  updateWidgets(); //GUIの表示を更新
  
  setCurrentFile(fileName);
  statusBar()->showMessage(tr("File loaded"), 2000);
}

bool MainWindow::saveFile(const QString &fileName){
  getParam(); //GUIから値を取得

  fr.write_file(fileName.toStdString(),true);
  
  setCurrentFile(fileName);
  statusBar()->showMessage(tr("File saved"), 2000);
  return true;
}

/************************************************************/
/*          Data Logging Routine                            */
/************************************************************/
int MainWindow::open_logfile(){
  //--- open log file ------
  MNP_LOG_FILE = fopen(data_filename,"w");

  for(int dof=0;dof<DOF;dof++ ){ // open axis data file
    AXIS_LOG_FILE[dof] =
      fopen( axis_data_filename[dof],"w");

  }
  return 0;
}

bool MainWindow::write_logfile(void* data,size_t size){
  LogBuf* buf = (LogBuf*)data;

  if( is_first_step() )
    set_time_offset(buf[0].time);  

  for(unsigned int i=0; i<size/sizeof(LogBuf); i++){
    //-- MNP DATA ------------------------------------------------
    fprintf(MNP_LOG_FILE, "%6.3f ",buf[i].time - time_offset() );

    for(int j=0; j<MNP_LOG_SIZE; j++)
      fprintf(MNP_LOG_FILE,  " %f", buf[i].mnp_data[j] );
    fprintf(MNP_LOG_FILE, "\n");

    //-- AXIS DATA -------------------
    for(int dof=0;dof<DOF;dof++ ){
      fprintf(AXIS_LOG_FILE[dof], "%6.3f ",buf[i].time - time_offset() );
      for(int j=0; j<AXIS_LOG_SIZE; j++)
	fprintf(AXIS_LOG_FILE[dof], " %f",buf[i].axis_data[j][dof]);
      fprintf(AXIS_LOG_FILE[dof], "\n");
    }// dof
    
  }// tempfile
  return true;
}

void MainWindow::close_logfile(){
  fclose(MNP_LOG_FILE);
  
  for(int dof=0;dof<DOF;dof++ )
    fclose(AXIS_LOG_FILE[dof]);

  Ktl::shell("mkdir -p %s",logdir );
  Ktl::shell("mv mnp*.dat* axis*.dat %s",logdir );
}
