/*
製作者：太田啓介
製作日：2024/11/28

スケッチの概要
 加速度を取得し，機体の現在のyawを計算
 TARGETに指定された目標のyawから離れると離れた量に対し，目標位置に戻すためのモータの回転量をPID補償器を介して計算
 回転量をpwm信号に変換し，DCモータドライバに出力

 倒立している間（±30°）はLEDが光り続ける．
*/

// M5系列の製品共通のライブラリ
#include <M5Unified.h>

// 目標位置のyaw
#define TARGET            -10.4
//PID係数
#define KP                10.0
#define KI                0.05
#define KD                0.0

// 倒立状態を示すLEDのピン番号をマクロで指定
#define LED_PIN 10

// DCモータドライバに接続するピン番号をマクロで指定
#define MOTOR_PIN_L_F        0    // 左モータの正転
#define MOTOR_PIN_L_R        26   // 左モータの逆転
#define MOTOR_PIN_R_F        32   // 右モータの正転
#define MOTOR_PIN_R_R        33   // 右モータの逆転

// PWM信号のチャンネルを指定
#define MOTOR_PWM_R_F        0    
#define MOTOR_PWM_R_R        1     
#define MOTOR_PWM_L_F        2     
#define MOTOR_PWM_L_R        3     

// PWM信号の最小値と最大値を指定 周波数を312500Hzにしているため，信号は8bitに分割．
#define MOTOR_POWER_MIN    50
#define MOTOR_POWER_MAX    255


// グローバル変数宣言 
/*
 powerはモータの回転量
 Iはyawの偏差の積分値
 prePは前ループのPの値
 preTimeは前ループの処理終了時間
 targetは目標位置TARGETを代入
*/
float power=0,I=0,preP=0,preTime;
float target = TARGET;

//IMUから加速度データを取得し，yawに変換する関数 引数はなしで戻り値はyawのfloat
float readGyro(){

  //ローカル変数指定
  /*
   acc_iは各軸の加速度
   pitch,rollは確認用に作成した変数
  */
  float accX,accY,accZ,pitch,roll,yaw;
  //M5Unified内ImuクラスのgetAccel関数によって加速度を取得．acc_iのポインタを引数にすることで,acc_iに値が代入される．
  M5.Imu.getAccel(&accX, &accY, &accZ);

  //yawの計算 確認用にroll，pitchの計算も行った．
  //roll  =  atan(accX / sqrt((accY * accY) + (accZ * accZ))) * 180 / 3.14; 
  //pitch =  atan(accY / sqrt((accX * accX) + (accZ * accZ))) * 180 / 3.14; 
  yaw   =  atan(sqrt((accX * accX) + (accY * accY)) / accZ) * 180 / 3.14;

  //yawは直立状態で90°付近のため，倒立状態で0°付近になるようにフォーマット
  if(yaw < 0){
    yaw +=90.0;
  }else{
    yaw -=90.0;
  }

  return yaw;
}

void setup(){
  
  //M5Unifiedクラスの初期化
  M5.begin();

  //LEDピン（10）の出力指定
  pinMode(LED_PIN, OUTPUT);
  
  //DCモータドライバに接続するピンの出力指定
  pinMode(MOTOR_PIN_R_F, OUTPUT);
  pinMode(MOTOR_PIN_R_R, OUTPUT);
  pinMode(MOTOR_PIN_L_F, OUTPUT);
  pinMode(MOTOR_PIN_L_R, OUTPUT);
  
  //pwm信号チャンネルの設定　（チャンネル名，周波数，bit数）
  ledcSetup(MOTOR_PWM_R_F, 312500, 8);
  ledcSetup(MOTOR_PWM_R_R, 312500, 8);
  ledcSetup(MOTOR_PWM_L_F, 312500, 8);
  ledcSetup(MOTOR_PWM_L_R, 312500, 8);
  
  //pwmチャンネルにピンを対応させる
  ledcAttachPin(MOTOR_PIN_L_F, MOTOR_PWM_L_F); //左モータ正転
  ledcAttachPin(MOTOR_PIN_L_R, MOTOR_PWM_L_R); //左モータ逆転
  ledcAttachPin(MOTOR_PIN_R_F, MOTOR_PWM_R_F); //右モータ正転
  ledcAttachPin(MOTOR_PIN_R_R, MOTOR_PWM_R_R); //右モータ逆転

  //処理開始時の時間を取得
  preTime = micros(); 
}


void loop() {
  //ローカル変数指定
  /*
   yaw（pitch，roll）はreadGyro関数で取得するyawの値
   Dutyは計算によって求めるpwm信号のDuty比 50~255の値を取る
   nowは目標yawとの現在のyawの偏差
   dtは前回ループから現在までの処理時間
   Pは目標yawと現在のyawの差を -1~+1 にした値
   Iは偏差Pの積分 ループ毎にPを加算
   Dは偏差の微分 前回ループのP（preP）と現在のPとの差を，前回ループから現在までの処理時間（dt）で割った値
   Timeは現在のループを開始した時間これを用いて1ループの処理時間dtを計算
  */
  float pitch,roll,yaw,Duty,P,D,now,dt,Time;

  //readGyro関数でyawを取得
  yaw = readGyro();

  //偏差の計算
  now     = target - yaw;

  //目標位置からの偏差が±30°の間は倒立を維持する処理を実行
  if (-30 < now && now < 30) { 
    //倒立している間はLEDを点灯
    digitalWrite(LED_PIN, LOW);

    //PIDの計算
    Time    = micros();
    dt      = (Time - preTime) / 1000000;
    preTime = Time;
    P       = now / 90;
    I      += P * dt;
    D       = (P - preP) / dt;
    preP    = P;
    //計算した偏差，積分値，微分値をそれぞれの係数KP，KI，KDに掛け合わせてモータの回転量を計算
    power  += KP * P + KI * I + KD * D;
    //powerが大きすぎ，または小さすぎる（絶対値が1を超える）場合は1に制限
    if (power < -1) power = -1;
    if (1 < power)  power =  1;

    //powerを50~255の値に変換し，Duty比の変数に代入
    Duty = (int)((MOTOR_POWER_MAX - 50 - MOTOR_POWER_MIN)* abs(power) + MOTOR_POWER_MIN);

    //pwmチャンネルに信号を送信
    //powerが正の場合は正転チャンネルに送信
    //powerが負の場合は逆転チャンネルに送信
    ledcWrite( MOTOR_PWM_R_F,(power < 0 ?    0 : Duty));
    ledcWrite( MOTOR_PWM_L_F,(power < 0 ?    0 : Duty));  
    ledcWrite( MOTOR_PWM_R_R,(power < 0 ? Duty :    0)); 
    ledcWrite( MOTOR_PWM_L_R,(power < 0 ? Duty :    0)); 

  } else {
    //偏差が±30°を超えたら倒れたとみなし，処理を実行しない

    //LEDを消灯
    digitalWrite(LED_PIN, HIGH);

    //モータの制御量を初期化
    power = 0;
    //偏差の累積値を初期化
    I = 0;
  }

  //目標位置targetをTARGET代入（バグ回避）
  target = TARGET;
}