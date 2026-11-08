
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

namespace C {
  constexpr int W=8, H=8;
  constexpr float WHEEL_DIA=42, WHEELBASE=90, CELL_SIZE=180;
  constexpr float ENCODER_CPR=0; // Calibrate!
  constexpr float TURN_SCALE=1.0;
  constexpr int WALL_MM=100, PWM=130, MIN_PWM=65, MAX_PWM=190;
  constexpr int LS=1, RS=1, LE=1, RE=1;
  constexpr uint32_t TIMEOUT=15000;
  constexpr uint8_t FA=0x30, LA=0x31, RA=0x32;
}

constexpr int AIN1=14, AIN2=23, PWMA=13;
constexpr int BIN1=16, BIN2=17, PWMB=32, STBY=33;
constexpr int LA=4, LB=5, RA=18, RB=19;
constexpr int SDA_PIN=21, SCL_PIN=22;
constexpr int XF=25, XL=26, XR=27, START=2;

volatile int32_t lt=0, rt=0;
volatile uint8_t pl=0, pr=0;
portMUX_TYPE emux=portMUX_INITIALIZER_UNLOCKED;

const int8_t qt[16]={
  0,-1,1,0, 1,0,0,-1,
  -1,0,0,1, 0,1,-1,0
};

void IRAM_ATTR leftISR(){
  uint8_t s=(digitalRead(LA)<<1)|digitalRead(LB);
  portENTER_CRITICAL_ISR(&emux);
  lt+=qt[(pl<<2)|s]*C::LE;
  pl=s;
  portEXIT_CRITICAL_ISR(&emux);
}

void IRAM_ATTR rightISR(){
  uint8_t s=(digitalRead(RA)<<1)|digitalRead(RB);
  portENTER_CRITICAL_ISR(&emux);
  rt+=qt[(pr<<2)|s]*C::RE;
  pr=s;
  portEXIT_CRITICAL_ISR(&emux);
}

void ticks(int32_t &l,int32_t &r){
  portENTER_CRITICAL(&emux);
  l=lt; r=rt;
  portEXIT_CRITICAL(&emux);
}

void motor(int a,int b,int p,int v,int sign){
  v*=sign;
  digitalWrite(a,v>0);
  digitalWrite(b,v<0);
  analogWrite(p,abs(v));
}

void drive(int l,int r){
  motor(AIN1,AIN2,PWMA,l,C::LS);
  motor(BIN1,BIN2,PWMB,r,C::RS);
}

void stopMotors(){ drive(0,0); }

float counts(float mm){
  return mm/(PI*C::WHEEL_DIA)*C::ENCODER_CPR;
}

bool moveWheels(int ld,int rd,float mm){
  if(C::ENCODER_CPR<=0) return false;

  int32_t sl,sr;
  ticks(sl,sr);
  int32_t target=counts(mm);
  uint32_t start=millis();

  while(true){
    int32_t l,r;
    ticks(l,r);
    int lp=(l-sl)*ld, rp=(r-sr)*rd;
    bool ldone=lp>=target, rdone=rp>=target;

    if(ldone&&rdone) break;
    if(millis()-start>C::TIMEOUT){
      stopMotors();
      return false;
    }

    int e=constrain((lp-rp)/3,-45,45);
    int lv=ldone?0:constrain(C::PWM-e,C::MIN_PWM,C::MAX_PWM);
    int rv=rdone?0:constrain(C::PWM+e,C::MIN_PWM,C::MAX_PWM);

    drive(ld*lv,rd*rv);
    delay(2);
  }

  stopMotors();
  delay(100);
  return true;
}

bool forwardCell(){
  return moveWheels(1,1,C::CELL_SIZE);
}

bool turnRight(){
  return moveWheels(1,-1,PI*C::WHEELBASE/4*C::TURN_SCALE);
}

bool turnLeft(){
  return moveWheels(-1,1,PI*C::WHEELBASE/4*C::TURN_SCALE);
}

bool turnBack(){
  return moveWheels(1,-1,PI*C::WHEELBASE/2*C::TURN_SCALE);
}

Adafruit_VL53L0X frontSensor,leftSensor,rightSensor;

bool initSensor(Adafruit_VL53L0X &s,int pin,uint8_t addr){
  digitalWrite(pin,HIGH);
  delay(10);
  return s.begin(addr,false,&Wire);
}

bool distance(Adafruit_VL53L0X &s,int &mm){
  VL53L0X_RangingMeasurementData_t m;
  s.rangingTest(&m,false);
  if(m.RangeStatus!=0) return false;
  mm=m.RangeMilliMeter;
  return true;
}

bool sensors(bool &f,bool &l,bool &r){
  int a,b,c;
  if(!distance(frontSensor,a)||
     !distance(leftSensor,b)||
     !distance(rightSensor,c)) return false;
  f=a<C::WALL_MM;
  l=b<C::WALL_MM;
  r=c<C::WALL_MM;
  return true;
}

enum Edge:uint8_t { UNKNOWN,OPEN,WALL };
enum Dir:uint8_t { N,E,S,W };

Edge maze[C::W][C::H][4];
const int dx[4]={0,1,0,-1};
const int dy[4]={1,0,-1,0};

int x=0,y=0;
Dir heading=N;

void setEdge(int cx,int cy,Dir d,Edge e){
  int nx=cx+dx[d],ny=cy+dy[d];
  maze[cx][cy][d]=e;
  if(nx>=0&&nx<C::W&&ny>=0&&ny<C::H)
    maze[nx][ny][(d+2)%4]=e;
}

Edge getEdge(int cx,int cy,Dir d){
  int nx=cx+dx[d],ny=cy+dy[d];
  if(nx<0||nx>=C::W||ny<0||ny>=C::H) return WALL;
  return maze[cx][cy][d];
}

void initMaze(){
  memset(maze,0,sizeof(maze));
  for(int i=0;i<C::W;i++)
    for(int j=0;j<C::H;j++){
      if(j==C::H-1)setEdge(i,j,N,WALL);
      if(i==C::W-1)setEdge(i,j,E,WALL);
      if(j==0)setEdge(i,j,S,WALL);
      if(i==0)setEdge(i,j,W,WALL);
    }
}

bool observe(){
  bool f,l,r;
  if(!sensors(f,l,r)) return false;
  setEdge(x,y,heading,f?WALL:OPEN);
  setEdge(x,y,(Dir)((heading+3)%4),l?WALL:OPEN);
  setEdge(x,y,(Dir)((heading+1)%4),r?WALL:OPEN);
  return true;
}

bool atGoal(){
  return (x==3||x==4)&&(y==3||y==4);
}

bool face(Dir d){
  int turn=(d-heading+4)%4;
  if(turn==1&&!turnRight())return false;
  if(turn==3&&!turnLeft())return false;
  if(turn==2&&!turnBack())return false;
  heading=d;
  return true;
}

bool moveTo(Dir d){
  int nx=x+dx[d],ny=y+dy[d];
  if(nx<0||nx>=C::W||ny<0||ny>=C::H)return false;
  if(getEdge(x,y,d)!=OPEN)return false;
  if(!face(d)||!forwardCell())return false;
  x=nx;y=ny;
  return true;
}

void failStop(){
  stopMotors();
  digitalWrite(STBY,LOW);
  Serial.println("FAULT");
  while(true)delay(100);
}

void solveRHS(){
  while(!atGoal()){
    if(!observe())failStop();

    Dir right=(Dir)((heading+1)%4);
    Dir left=(Dir)((heading+3)%4);
    Dir back=(Dir)((heading+2)%4);
    Dir next;

    if(getEdge(x,y,right)==OPEN) next=right;
    else if(getEdge(x,y,heading)==OPEN) next=heading;
    else if(getEdge(x,y,left)==OPEN) next=left;
    else if(getEdge(x,y,back)==OPEN) next=back;
    else failStop();

    if(!moveTo(next))failStop();

    Serial.printf("(%d,%d) heading=%d\n",x,y,heading);
  }

  stopMotors();
  digitalWrite(STBY,LOW);
  Serial.println("CENTRE REACHED");
}

void setup(){
  Serial.begin(115200);

  int outputs[]={AIN1,AIN2,PWMA,BIN1,BIN2,PWMB,STBY};
  for(int p:outputs)pinMode(p,OUTPUT);
  digitalWrite(STBY,LOW);

  pinMode(START,INPUT_PULLUP);

  pinMode(LA,INPUT_PULLUP);
  pinMode(LB,INPUT_PULLUP);
  pinMode(RA,INPUT_PULLUP);
  pinMode(RB,INPUT_PULLUP);

  pl=(digitalRead(LA)<<1)|digitalRead(LB);
  pr=(digitalRead(RA)<<1)|digitalRead(RB);

  attachInterrupt(digitalPinToInterrupt(LA),leftISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB),leftISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RA),rightISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RB),rightISR,CHANGE);

  pinMode(XF,OUTPUT);
  pinMode(XL,OUTPUT);
  pinMode(XR,OUTPUT);
  digitalWrite(XF,LOW);
  digitalWrite(XL,LOW);
  digitalWrite(XR,LOW);

  Wire.begin(SDA_PIN,SCL_PIN);

  if(!initSensor(frontSensor,XF,C::FA))failStop();
  if(!initSensor(leftSensor,XL,C::LA))failStop();
  if(!initSensor(rightSensor,XR,C::RA))failStop();

  initMaze();

  if(C::ENCODER_CPR<=0){
    Serial.println("Set ENCODER_CPR first!");
    failStop();
  }

  Serial.println("Dedcat ready");
}

void loop(){
  if(digitalRead(START)==LOW){
    delay(50);
    if(digitalRead(START)==LOW){
      digitalWrite(STBY,HIGH);
      delay(100);
      solveRHS();
      while(true)delay(100);
    }
  }
}

