/*
motors with 200 step resolution
8 or 16 microsteps
transmission ratio [1:5, 1:10, 1:12.6]
1.125 deg per trigger event
*/

#include <EEPROM.h> // read/write specific turntable data
#include <Wire.h>   // I2C communication
#include <LSM303.h> // accelerometer library

// I/O pins
#define OTT1 0 
#define OTT2 1
#define nSLP_h 4
#define STP_h 5
#define DIR_h 6
#define BUTTON 7
#define nSLP_v 11 
#define STP_v 12
#define DIR_v 13

#define SENSOR 2 // analog input
#define THRESHOLD_SENSOR 100

// memory addresses
#define EEPROM_ADDR_TRANS_h 0
#define EEPROM_ADDR_TRANS_v 1
#define EEPROM_ADDR_mSTEPS_h 2
#define EEPROM_ADDR_mSTEPS_v 3
#define EEPROM_ADDR_OFFSET_h 5
#define EEPROM_ADDR_OFFSET_v 6

// global variables
bool active_h, active_v, is_3D = false;
bool button_pushed = false;
int x, y, val = 0;
double delvar;
char serialBuffer[128];
LSM303 accelerometer;

int msteps_h, msteps_v;
float transmission_h, transmission_v;
signed int offset_h, offset_v;  // marker/accelerometer offset
float spd_h, spd_v;  // steps-per-degree
int trig_step_width_h, trig_step_width_v; // triggering from other turntable
float angle_h_is, angle_v_is;
float angle_h_target, angle_v_target;

unsigned long last_interrupt_time = 0;

void setup() {
  pinMode(nSLP_h, OUTPUT); // Set I/O pins
  pinMode(STP_h, OUTPUT);
  pinMode(DIR_h, OUTPUT);
  pinMode(nSLP_v, OUTPUT);
  pinMode(STP_v, OUTPUT);
  pinMode(DIR_v, OUTPUT);
  digitalWrite(nSLP_h, LOW); // keep motors disabled
  digitalWrite(nSLP_v, LOW);

  pinMode(OTT1, INPUT_PULLUP); // From other Turntable
  pinMode(OTT2, OUTPUT);       // To other Turntable
  digitalWrite(OTT2, HIGH);
  
  pinMode(BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON), button, FALLING);

  Serial.begin(9600); // Open Serial connection
  Wire.begin();
  
  int trans_tmp = EEPROM.read(EEPROM_ADDR_TRANS_h);
  if (trans_tmp != 0xFF) {
    transmission_h = (float)trans_tmp / 10;
  }
  trans_tmp = EEPROM.read(EEPROM_ADDR_TRANS_v);
  if (trans_tmp != 0xFF) {
    is_3D = true;
    transmission_v = (float)trans_tmp / 10;
  }
  int msteps_tmp = EEPROM.read(EEPROM_ADDR_mSTEPS_h);
  if (msteps_tmp != 0xFF) {
    msteps_h = msteps_tmp;
  }
  msteps_tmp = EEPROM.read(EEPROM_ADDR_mSTEPS_v);
  if (msteps_tmp != 0xFF) {
    msteps_v = msteps_tmp;
  }
  int offset_tmp = EEPROM.read(EEPROM_ADDR_OFFSET_h);
  if (offset_tmp != 0xFF) {
    offset_h = offset_tmp - 128;
  }
  offset_tmp = EEPROM.read(EEPROM_ADDR_OFFSET_v);
  if (offset_tmp != 0xFF) {
    offset_v = offset_tmp - 128;
  }  
  calculate();
}

void loop() {
  if (button_pushed) {
    calibrate();
    button_pushed = false;
  }
  //int zahl = analogRead(SENSOR);
  //Serial.println(zahl);
  if (!active_h && analogRead(SENSOR) < THRESHOLD_SENSOR) {
    active_h = true;
    digitalWrite(nSLP_h, HIGH); // set controller active
  }

  if (Serial.available()) {
    parse();
  }
  else {
    val = digitalRead(OTT1);
    if (val == 0) {
      Step(trig_step_width_h, STP_h);
      delay(2); //make sure to not catch the same trigger again
    }
  }
}

void button() {
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 1000){
    button_pushed = true;
    last_interrupt_time = interrupt_time;
  }
}

void calculate()
{
  spd_h = transmission_h * msteps_h / 1.8;
  trig_step_width_h = 10 * transmission_h;
  spd_v = msteps_v * transmission_v / 1.8;
  trig_step_width_v = 10 * transmission_v;
}

// return to vertical equilibrium
void calibrate()
{
  if (!active_v){
    active_v = true;
    digitalWrite(nSLP_v, HIGH);
    accelerometer.init();
    accelerometer.enableDefault(); // enable data stream
  }
  else accelerometer.read();
  int steps = (asin((float)accelerometer.a.x / 16384) * 180 / 3.141 + offset_v) * spd_v;
  
  //Serial.println(steps);
  //if (abs(steps) > 1500) steps = 0;
  if (steps < 0) {
    digitalWrite(DIR_v, LOW);
    steps = abs(steps);
  }
  else {
    digitalWrite(DIR_v, HIGH);
  }
  if (steps != 0) {
    Step(steps, STP_v);
  }
}

void parse() {
  byte num = Serial.readBytesUntil('\n', serialBuffer, 128);
  if (num == 0) {
    return;
  }
  char* tok = strtok(serialBuffer, " \r");
  if (tok == 0) {
    return;
  }

  if (strcmp(tok, "help") == 0) {
    Serial.println("get trig_width : get steps per trigger input");
    Serial.println("get spd : get steps per degree");
    Serial.println("get trans : get transmission ratio");
    Serial.println("set trans x: set transmission ratio");
    Serial.println("set msteps x: set microSteps of motorcontroller");
    Serial.println("move x: move x degrees");
    Serial.println("trig : trigger remote table");
    if (is_3D) {
      Serial.println("-------------------------");
      Serial.println("use suffix '_h' for horizontal and '_v' for vertical control");
      Serial.println("eg: 'move_v x' or 'set msteps_h x'");
    }
  }
  if (strcmp(tok, "sleep") == 0) {
    active_h = active_v = false;
    digitalWrite(nSLP_h, LOW);
    digitalWrite(nSLP_v, LOW);
  }
  if (strcmp(tok, "get") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      return;
    }
    if (strcmp(tok2, "spd") == 0 || strcmp(tok2, "spd_h") == 0) {
      Serial.println(spd_h);
    }
    if (strcmp(tok2, "spd_v") == 0) {
      Serial.println(spd_v);
    }
    if (strcmp(tok2, "trig_width") == 0 || strcmp(tok2, "trig_width_h") == 0) {
      Serial.println(trig_step_width_h);
    }
    if (strcmp(tok2, "trig_width_v") == 0) {
      Serial.println(trig_step_width_v);
    }
    if (strcmp(tok2, "trans") == 0 || strcmp(tok2, "trans_h") == 0) {
      Serial.println(transmission_h);
    }
    if (strcmp(tok2, "trans_v") == 0) {
      Serial.println(transmission_v);
    }
    if (strcmp(tok2, "msteps") == 0 || strcmp(tok2, "msteps_h") == 0) {
      Serial.println(msteps_h);
    }
    if (strcmp(tok2, "msteps_v") == 0) {
      Serial.println(msteps_v);
    }
    if (strcmp(tok2, "offset") == 0 || strcmp(tok2, "offset_h") == 0) {
      Serial.println(offset_h);
    }
    if (strcmp(tok2, "offset_v") == 0) {
      Serial.println(offset_v);
    }
  }
  if (strcmp(tok, "set") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      return;
    }
    if (strcmp(tok2, "trans") == 0 || strcmp(tok2, "trans_h") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        return;
      }
      char* next;
      transmission_h = strtod(tok3, &next);
      EEPROM.update(EEPROM_ADDR_TRANS_h, transmission_h * 10);
    }
    if (strcmp(tok2, "trans_v") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        return;
      }
      char* next;
      transmission_v = strtod(tok3, &next);
      EEPROM.update(EEPROM_ADDR_TRANS_v, transmission_v * 10);
    }
    if (strcmp(tok2, "msteps") == 0 || strcmp(tok2, "msteps_h") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        return;
      }
      char* next;
      msteps_h = strtol(tok3, &next, 10);
      EEPROM.update(EEPROM_ADDR_mSTEPS_h, msteps_h);
    }
    if (strcmp(tok2, "msteps_v") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        return;
      }
      char* next;
      msteps_v = strtol(tok3, &next, 10);
      EEPROM.update(EEPROM_ADDR_mSTEPS_v, msteps_v);
    }
    if (strcmp(tok2, "offset_h") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        return;
      }
      char* next;
      offset_v = strtol(tok3, &next, 10) ;
      EEPROM.update(EEPROM_ADDR_OFFSET_h, offset_h + 128);
    }
    if (strcmp(tok2, "offset_v") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        return;
      }
      char* next;
      offset_v = strtol(tok3, &next, 10) ;
      EEPROM.update(EEPROM_ADDR_OFFSET_v, offset_v + 128);
    }
    calculate();
    Serial.println("OK");
  }

  if (strcmp(tok, "move") == 0 || strcmp(tok, "move_h") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      return;
    }
    char* next;
    int angle = strtol((const char*)tok2, &next, 10);
    if (angle < 0) {
      angle = abs(angle);
      if (is_3D) digitalWrite(DIR_h, HIGH); // horizontal motors are flipped
      else digitalWrite(DIR_h, LOW);      
    }
    else {
      if (is_3D) digitalWrite(DIR_h, LOW);
      else digitalWrite(DIR_h, HIGH);
    }
    if (angle != 0) {
      int steps = angle * spd_h;
      Serial.println(steps);
      Step(steps, STP_h);
    }
  }

  if (strcmp(tok, "move_v") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      return;
    }
    char* next;
    int angle = strtol((const char*)tok2, &next, 10);
    if (angle < 0) {
      digitalWrite(DIR_v, LOW);
      angle = abs(angle);
    }
    else {
      digitalWrite(DIR_v, HIGH);
    }
    if (angle != 0) {
      int steps = angle * spd_v;
      Step(steps, STP_v);
    }
  }
  if (strcmp(tok, "trig") == 0) {
    digitalWrite(OTT2, LOW);
    delay(1); // 1ms
    digitalWrite(OTT2, HIGH);
    Serial.println("OK");
  }
}

void Step(int steps, int pin)
{
  delvar = 6000;
  for (x = 1; x <= steps; x++)  {
    if (button_pushed) {
      button_pushed = false;
      break;
    }
    digitalWrite(pin, HIGH); //Trigger one step forward
    delayMicroseconds(delvar);
    digitalWrite(pin, LOW);
    delayMicroseconds(delvar);
    if (delvar > 150 && x < steps / 2) {
      delvar -= 2 * delvar / (4 * x + 1);
    }
    if ((steps > 1752 && x > steps - 876) || (steps <= 1752 && x >= steps / 2)) {
      delvar -= 2 * delvar / (4 * (x - steps - 1) + 1);
    }  
  }
  
  // return x
  // add x to real angle aber steps.... mit spd
  Serial.println("OK");
}
