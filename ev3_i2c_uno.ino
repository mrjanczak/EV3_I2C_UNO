// I2C Slave Send / Receive
// A4 – SDA
// A5 – SCL
// See www.dexterindustries.com/howto for more information on the physical setup.

int instruction[5] = {6, 0, 0, 0, 0};
/// instruction[0] = 1 (LED), 2 (servo motor} ,3 (DC motor), 4 (sensor), 5 (DC motor by revs)
///
/// instruction [0] = 1 ==>  instruction [1] is port (LED digital pin)
///                          instruction [2] is: 0 (LED off) or 1 (LED on)
///
/// instruction [0] = 2 ==>  instruction [1] is port (servo motor number)
///                          instruction [2] is angle (0-180)
///
/// instruction [0] = 3 ==>  instruction [1] is DC motor number
///                          instruction [2] is power (-100 +100)
///
/// instruction [0] = 4 ==>  instruction [1] is pin number
///                          instruction [2] is type: 0 (analog pin) or 1 (digital pin)
///
/// instruction [0] = 5 ==>  instruction [1] is DC motor number
///                          instruction [2] is cumulative number of revolvs

#define I2C_ADDRESS 0x04
#include<Wire.h> // I2C library
byte read_byte = 0x00;
int byte_count = 0;
int temp_sensor = 0;

#include <Servo.h>
Servo temp_servo;

#define DEF_SPEED 255
#define M_COUNT 4
#define V_SIZE 20
#define DELAY_TIME 100
#include <AFMotor.h>
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
int dir[M_COUNT], curr_rev[M_COUNT], des_rev[M_COUNT], next_rev[M_COUNT], v[V_SIZE*M_COUNT], v_idx[M_COUNT];
int  m_sign[M_COUNT]  = {-1,+1,+1,+1};
bool m_check[M_COUNT];

#define THRESHOLD 800
#include <RedBot.h>
RedBotSensor LSleft  = RedBotSensor(A0);
RedBotSensor LScent  = RedBotSensor(A1);
RedBotSensor LSright = RedBotSensor(A2);

void setup()
{
  Wire.begin(I2C_ADDRESS); // set the slave address
  Wire.onRequest(requestEvent); // Sending information back to the NXT/EV3
  Wire.onReceive(receiveI2C); // Receiving information!

  Serial.begin(9600);

  int p;
  for (p = 0; p < M_COUNT; p++) {
    dir[p] = 0;
    curr_rev[p] = 0;
    des_rev[p]  = 0; 
    next_rev[p] = -1; 
    m_check[p]  = 0; 
    v_idx[p]    = 0;
    for (int i = 0; i < V_SIZE; i++) 
      v[(p-1)*V_SIZE + i] = 1;   //history of linesensor checks  
    setMotor(p,dir[p],0);  
  }
}

void loop()
{
  delay(DELAY_TIME); //~40 samples per rev
  
  motorBySerial(0);  // for debugging only
  
  int p;
  for (p = 0; p < M_COUNT; p++) {
    if (m_check[p]) {
      checkMotor(p); 
      //getNextRev(p);
    }
  } 
}

void setMotor(int p, int d, int s)
{
  d *= m_sign[p]; // direction consistent with motor sign
  switch (p) {
    case 0:
      if (d < 0 ) { motor1.setSpeed(s); motor1.run(BACKWARD); }
      if (d > 0 ) { motor1.setSpeed(s); motor1.run(FORWARD); }
      if (d == 0) { motor1.run(RELEASE); } 
      break;
    case 1:
      if (d < 0 ) { motor2.setSpeed(s); motor2.run(BACKWARD); }
      if (d > 0 ) { motor2.setSpeed(s); motor2.run(FORWARD); }
      if (d == 0) { motor2.run(RELEASE); } 
      break;
    case 2:
      if (d < 0 ) { motor3.setSpeed(s); motor3.run(BACKWARD); }
      if (d > 0 ) { motor3.setSpeed(s); motor3.run(FORWARD); }
      if (d == 0) { motor3.run(RELEASE); } 
      break;
    case 3:
      if (d < 0 ) { motor4.setSpeed(s); motor4.run(BACKWARD); }
      if (d > 0 ) { motor4.setSpeed(s); motor4.run(FORWARD); }
      if (d == 0) { motor4.run(RELEASE); } 
      break;  
  }
}

bool checkLineSensor(int p) {
  switch (p) {
    case 0:
      return LSleft.read() > THRESHOLD;
      break;
    case 1:
      return LSright.read() > THRESHOLD;
      break; 
    default:
      return false;
      break;           
  }
}

void checkMotor(int p)
{
  int i, prev_v_sum = 0, curr_v = checkLineSensor(p);
  for (i = 0; i < V_SIZE; i++) prev_v_sum += v[(p-1)*V_SIZE + i];
  //append current check to checks history only if motor rotates (dir!=0)
  if (dir[p] != 0) {
    v[(p-1)*V_SIZE + v_idx[p]] = curr_v;
    v_idx[p]++; 
    if (v_idx[p] == V_SIZE) v_idx[p] = 0;
  }
  Serial.println(curr_v);
  
  //detect new pass over sensor after V_SIZE empty checks
  if ((dir[p] != 0) && (prev_v_sum == 0) && (curr_v == 1)) {
    curr_rev[p] += dir[p]; //count new reverse
    Serial.print("Completed rev:"); Serial.println(curr_rev[p]);
  }

  // Release motor if desired rev completed
  if (des_rev[p] == curr_rev[p]) {
    setMotor(p,0,0);
    dir[p] = 0;
  }
}

void getNextRev(int p) {
  // if completed current task, starts the next one
  if ((dir[p] == 0) && (next_rev[p] >= 0)) {
      des_rev[p]  = next_rev[p];
      next_rev[p] = -1;       
      dir[p] = sign(des_rev[p] - curr_rev[p]);
      setMotor(p, dir[p], DEF_SPEED); 
  }   
}
  
// When data is received from NXT/EV3, this function is called.
void receiveI2C(int bytesIn)
{
  int x,comm, p, state, angle, motor_speed, rev, a_d;
  read_byte = bytesIn;
  byte_count = 0;

  while (1 < Wire.available()) // loop through all but the last
  {
    read_byte = Wire.read();
    instruction[byte_count] = read_byte;
    byte_count++;
  }
  x = Wire.read(); // Read the last dummy byte (has no meaning, but must read it)
  comm = instruction[0];
  
  if ( comm == 1 )
  {
    p = instruction[1];
    state = instruction[2];
    Serial.println("LED ");
    Serial.print("  Pin: ");   Serial.println( p );
    Serial.print("  State: "); Serial.println( state );
    pinMode(p, OUTPUT);
    if (state == 0) digitalWrite(p, LOW);
    else  digitalWrite(p, HIGH);
  }
  else if ( comm == 2 )
  {
    p = instruction[1];
    angle = instruction[2] * 2 + instruction[3];
    Serial.println("Servo Motor ");
    Serial.print("  Pin: ");   Serial.println( p );
    Serial.print("  Angle: "); Serial.println( angle );
    temp_servo.attach(p);
    temp_servo.write(angle);
  }
  else if ( comm == 3 )
  {
    p = instruction[1]-1; //offset motor # by -1
    motor_speed = (instruction[2] - 50) * 2 * 2.55;
    Serial.println("DC Motor by speed");
    Serial.print("  Port: ");  Serial.println( p );
    Serial.print("  Speed: "); Serial.println( motor_speed );

    m_check[p] = 0;
    setMotor(p,sign(motor_speed),abs(motor_speed));
  }
  else if ( comm == 4 )
  {
    p = instruction[1]-1; //offset motor # by -1
    rev = instruction[2];
    Serial.println("DC Motor by revs");
    Serial.print("  Port: " );  Serial.println( p );
    Serial.print("  Revs: " );  Serial.println( rev );

    m_check[p] = 1;
    int temp_dir = sign(rev - curr_rev[p]);
    if ((dir[p] == 0) || (dir[p]*temp_dir > 0))
      des_rev[p] = rev;                 // execute new rev if starts or continues rotation
    else 
      next_rev[p] = rev;                // wait with execution until completes prev. rev
        
    dir[p] = sign(des_rev[p] - curr_rev[p]);
    setMotor(p, dir[p], DEF_SPEED);
  }
  else if ( comm == 5 )
  {
    p = instruction[1];
    a_d = instruction[2];
    Serial.println("Sensor ");
    Serial.print("  Pin: " );  Serial.print(p); Serial.print(":"); Serial.println(a_d);

    if ( a_d == true ) {
      int temp_pin = A0;
      if (p != 0 )  temp_pin += p;
      temp_sensor = analogRead(temp_pin);
    }
    else {
      pinMode(p, INPUT);
      temp_sensor = digitalRead(p);
    }
  }

}//end recieveI2C

//________________________________________________________________________________

void requestEvent()
{
  if (instruction[0] == 4)
  {
    Wire.write(temp_sensor); // respond with message
    Serial.print("Value: ");
    Serial.println(temp_sensor);
  }
}//end requestEvent
//________________________________________________________________________________

void motorBySerial(int p)
{
  if (Serial.available())
  {
    int rev = Serial.parseInt();    
    m_check[p] = 1;
    /*
    int temp_dir = sign(rev - curr_rev[p]);
    if ((dir[p] == 0) || (dir[p]*temp_dir > 0))
      des_rev[p] = rev;                 // execute new rev if starts or continues rotation
    else 
      next_rev[p] = rev;                // wait with execution until completes prev. rev
    */    
    des_rev[p] = rev; // test mode
    dir[p] = sign(des_rev[p] - curr_rev[p]);
    setMotor(p, dir[p], DEF_SPEED);

    Serial.print("des rev:");      Serial.print(des_rev[p]);
    Serial.print(" | curr rev:");  Serial.println(curr_rev[p]);
  }
}

static inline int8_t sign(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}
