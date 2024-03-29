#ifndef __DEF__
#define __DEF__

enum Command{
  COMMAND_SERVO_ON, //電磁弁を開き空気を供給し駆動力制御を開始する
  COMMAND_SERVO_OFF,//供給空気を排気し脱力状態にする
  COMMAND_START, //位置制御を開始し司令を待つ
  COMMAND_STOP,  //位置制御を停止する
  COMMAND_MOVE_TO, //手先を指定位置へ移動させる
  COMMAND_ABORT,//SERVO_OFFと同じ.緊急用として任意の状態から脱力状態に遷移する
  COMMAND_LAST
};

const char* command_text[]={
  "SERVO_ON",
  "SERVO_OFF",
  "START",
  "STOP",
  "MOVE_TO",
  "ABORT"
};
  

enum State{
  STATE_SERVO_OFF,
  //電磁弁により空気が排気され脱力している状態

  STATE_STANDBY,
  //空気が供給され駆動力制御されている状態、受動状態

  STATE_READY,
  //手先の位置姿勢が制御され、司令を受け付ける状態

  STATE_TASK_RUNNING,
  //司令されたタスクを実行している状態・タスクが正常終了するとREADY状態に戻る

  STATE_TEST_RUNNING,
  //テスト動作中．開発用
};

const char* state_text[]={
  "SERVO_OFF",
  "STANDBY",
  "READY",
  "TASK_RUNNING",
  "TEST_RUNNING",
};


enum Result{
  RESULT_FAILURE = -1,
  RESULT_TRYING = 0,
  RESULT_SUCCESS = 1
};

#endif



