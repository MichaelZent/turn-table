/*
  Pushing the button can be used to interrupt any motor movement!
  Startup sequence:
  - Push the button to enable the vertical motor
  - Push the button again to balance the box
  - Turn until the marker aligns with the distance sensor to enable the horizontal motor

  -Vertical: positive Angle turns antenna up
*/

#include <EEPROM.h> // read/write specific turntable data
#include <Wire.h>   // I2C communication

// I/O pins
#define OTT1 0
#define OTT2 1
#define REED 2
#define nSLP_h 4
#define STP_h 5
#define DIR_h 6
#define BUTTON 7
#define nSLP_v 11
#define STP_v 12
#define DIR_v 13

#define SENSOR A2 // analog input
#define THRESHOLD_SENSOR 100

// memory addresses
#define EEPROM_ADDR_TRANS_h 0
#define EEPROM_ADDR_TRANS_v 1
#define EEPROM_ADDR_mSTEPS_h 2
#define EEPROM_ADDR_mSTEPS_v 3
#define EEPROM_ADDR_OFFSET_h 5
#define EEPROM_ADDR_OFFSET_v 6

// global variables
bool active_h, active_v, is_2D = false;
bool button_pushed = false;
int x, y, val = 0;
float delvar;


int msteps_h, msteps_v;
float transmission_h, transmission_v;
signed int offset_h, offset_v;  // marker/accelerometer offset
float spd_h, spd_v;  // steps-per-degree
int trig_step_width_h, trig_step_width_v; // triggering from other turntable
float angle_h_is = 0;
float angle_v_is = 0;
float angle_h_target = 0;
float angle_v_target = 0;

unsigned long last_interrupt_time = 0;

void setup() {
  pinMode(nSLP_h, OUTPUT); // Set I/O pins
  pinMode(STP_h, OUTPUT);
  pinMode(DIR_h, OUTPUT);
  pinMode(nSLP_v, OUTPUT);
  pinMode(STP_v, OUTPUT);
  pinMode(DIR_v, OUTPUT);
  pinMode(REED, INPUT_PULLUP);
  digitalWrite(nSLP_h, LOW); // keep motors disabled
  digitalWrite(nSLP_v, LOW);
  digitalWrite(STP_v, LOW); // initial start
  
  

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
    Serial.println(transmission_h);
  }
  trans_tmp = EEPROM.read(EEPROM_ADDR_TRANS_v);
  if (trans_tmp != 0xFF) {
    is_2D = true;
    transmission_v = (float)trans_tmp / 10;
    Serial.println(transmission_v);
  }
  int msteps_tmp = EEPROM.read(EEPROM_ADDR_mSTEPS_h);
  if (msteps_tmp != 0xFF) {
    msteps_h = msteps_tmp;
    Serial.println(msteps_h);
  }
  msteps_tmp = EEPROM.read(EEPROM_ADDR_mSTEPS_v);
  if (msteps_tmp != 0xFF) {
    msteps_v = msteps_tmp;
    Serial.println(msteps_v);
  }
  int offset_tmp = EEPROM.read(EEPROM_ADDR_OFFSET_h);
  if (offset_tmp != 0xFF) {
    offset_h = offset_tmp;
    Serial.println(offset_h);
  }
  offset_tmp = EEPROM.read(EEPROM_ADDR_OFFSET_v);
  if (offset_tmp != 0xFF) {
    offset_v = offset_tmp - 128;
    Serial.println(offset_v);
  }
  spd_h = transmission_h * msteps_h / 1.8;
  trig_step_width_h = 10 * transmission_h;
  spd_v = msteps_v * transmission_v / 1.8;
  trig_step_width_v = 10 * transmission_v;

  Serial.println(spd_h);
  Serial.println(trig_step_width_h);
  Serial.println(spd_v);
  Serial.println(trig_step_width_v);
  
}

// ISR: set global variable
void button()
{
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 1000)
  {
    button_pushed = true;
    last_interrupt_time = interrupt_time;
  }
}

void loop() {

  if (digitalRead(REED)==1)
{
  Serial.println("1");
  delay(500);
  }
if(digitalRead(REED)==0)
{
  Serial.println("0");
  delay(500);
}

  if (button_pushed) {
    button_pushed = false;
    Serial.write("Button pushed");
    if (!active_v) {
      active_v = true;
      digitalWrite(nSLP_v, HIGH);
    }
  }

  //balance();






  if (!active_h && analogRead(SENSOR) < THRESHOLD_SENSOR) {
    active_h = true;
    Serial.println("horizontal motor activated");
    digitalWrite(nSLP_h, HIGH); // set controller active
    digitalWrite(DIR_h, HIGH); // move clockwise
    delay(100);
    //Step((offset_h * spd_h), STP_h); // go to 0 degrees
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


// return to vertical equilibrium (offset_v)
void balance()
{
  if (!active_v) {
    active_v = true;
    digitalWrite(nSLP_v, HIGH);
    Serial.println("Vertical motor activated");
  }

if (digitalRead(REED) == LOW) {

  digitalWrite(DIR_v, HIGH);
  delvar = 6000;
  while(digitalRead(REED) == LOW) {

      
    digitalWrite(STP_v, HIGH);
    delayMicroseconds(delvar);
    digitalWrite(STP_v, LOW);
    delayMicroseconds(delvar);
  }

  digitalWrite(DIR_v, LOW);
  int steps = 16 * spd_v; //tbd
  Step(steps, STP_v);

  angle_v_is = 0.0;
  Serial.println("Vertical equilibrium reached. Equilibrium is Angle_v_is = 0.");  
}
else {

    digitalWrite(DIR_v, HIGH);
  delvar = 6000;
   Serial.println("Adjusting vertical angle. If antenna is higher than 45 degree push the button!");
while( digitalRead(REED) == HIGH) {


      if(button_pushed){
        button_pushed = false;
        return;
      }
     digitalWrite(STP_v, HIGH);
    delayMicroseconds(delvar);
    digitalWrite(STP_v, LOW);
    delayMicroseconds(delvar);
    
}
balance();
      
}
  


}



// execute command from user
void parse() {
  char serialBuffer[128] = {0};
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
    Serial.println("activate: activate motor");
    Serial.println("deactivate: deactivate motor");
    Serial.println("sleep: deactivate all motors");
    Serial.println("status: get current position");
    if (is_2D) {
      Serial.println("-------------------------");
      Serial.println("use suffix '_h' for horizontal and '_v' for vertical control");
      Serial.println("eg: 'move_v x' or 'set msteps_h x'");
    }
  }
  if (strcmp(tok, "sleep") == 0) {
    active_h = active_v = false;
    Serial.write("Motors deacitvated");
    digitalWrite(nSLP_h, LOW);
    digitalWrite(nSLP_v, LOW);
  }
  if (strcmp(tok, "activate_h") == 0 || strcmp(tok, "activate") == 0) {
    active_h = true;
    digitalWrite(nSLP_h, HIGH);
    Serial.println("horizontal motor activated");
  }
  if (strcmp(tok, "activate_v") == 0) {
    active_v = true;
    digitalWrite(nSLP_v, HIGH);
    Serial.println("vertical motor activated");
  }
  if (strcmp(tok, "deactivate_h") == 0 || strcmp(tok, "deactivate") == 0)  {
    active_h = false;
    digitalWrite(nSLP_h, LOW);
    Serial.println("horizontal motor deactivated");
  }
  if (strcmp(tok, "deactivate_v") == 0) {
    active_v = false;
    digitalWrite(nSLP_v, LOW);
    Serial.println("vertical motor deactivated");
  }
  if (strcmp(tok, "balance_v") == 0) {
    balance();
  }
  if (strcmp(tok, "status") == 0) {
    Serial.println(angle_h_is);
    if(is_2D) {
    Serial.println(angle_v_is);
    }
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
      offset_h = strtol(tok3, &next, 10) ;
      EEPROM.update(EEPROM_ADDR_OFFSET_h, offset_h);
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
    Serial.println("OK");
  }

  // limited between 0 and 360°
  if (strcmp(tok, "move") == 0 || strcmp(tok, "move_h") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      return;
    }
    if(!active_h) {
      Serial.println("Error: Motor is deactivated");
      return;
    }
    char* next;
    int steps = 0;
    double angle = strtod((const char*)tok2, &next);

    if (angle > 360 || angle <-360) {
      Serial.println("Error: Rotation angle over 360 degree");
      return;
    }
    angle_h_target += angle;
    Serial.println("Target angle: ");
    Serial.println(angle_h_target);
    //limit the target angle between 0° and 360°
    if (is_2D && (angle_h_target > 360 || angle_h_target < 0)) {

      if (angle_h_target > 360) {

        angle_h_target = angle_h_target - 360;
        
      }

      if (angle_h_target < 0) {

        angle_h_target = angle_h_target + 360;
      }
    }
    
      //angle = angle_h_target - angle_h_is;
          Serial.println("Target angle: ");
    Serial.println(angle_h_target);
        Serial.println("Angle: ");
    Serial.println(angle);
      if (angle < 0) {
        angle = abs(angle);
        if (is_2D) digitalWrite(DIR_h, HIGH); // horizontal motors are flipped ??
        else digitalWrite(DIR_h, LOW);
        steps = angle * spd_h;
        angle_h_is -= Step(steps, STP_h) / spd_h;
        if (angle_h_is <0) {
          angle_h_is += 360;
        }
      Serial.print("New Angle_h: ");
      Serial.println(angle_h_is);
      }
      else {
        if (is_2D) digitalWrite(DIR_h, LOW);
        else digitalWrite(DIR_h, HIGH);
        steps = angle * spd_h;
        angle_h_is += Step(steps, STP_h) / spd_h;
        
              if (angle_h_target > 360) {

        angle_h_target -=  360;
        
      }
        
        Serial.print("New Angle_h: ");
      Serial.println(angle_h_is);
      }
      Serial.print("Driven Steps_h: ");
      Serial.println(steps);
    }
  

  // limited between -46° and +46°
  if (strcmp(tok, "move_v") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      return;
    }
    if(!active_v){
      Serial.println("Error: Motor is deactivated!");
      return;
    }
    char* next;
    int steps;
    double angle = strtod((const char*)tok2, &next);
    Serial.println(angle);
    angle_v_target += angle;
    Serial.print("Target Agngle_v: ");
    Serial.println(angle_v_target);
    
    if (angle_v_target > 46 || angle_v_target < -46) {
      angle_v_target -= angle;
      Serial.println("Not OK");
      Serial.println(angle_v_target);
      Serial.println(angle);
    }
    else {
      angle = angle_v_target - angle_v_is;
      if (angle < 0) {
        digitalWrite(DIR_v, LOW);
        angle = abs(angle);
        steps = angle * spd_v;
        angle_v_is -= Step(steps, STP_v) / spd_v;
      Serial.print("New Angle_v: ");
      Serial.println(angle_v_is);
      }
      else {
        digitalWrite(DIR_v, HIGH);
        steps = angle * spd_v;
        angle_v_is += Step(steps, STP_v) / spd_v;
      Serial.print("New Angle_v: ");
      Serial.println(angle_v_is);
      }
      Serial.print("Driven Steps_v: ");
      Serial.println(steps);
    }
  }
  if (strcmp(tok, "trig") == 0) {
    digitalWrite(OTT2, LOW);
    delay(1); // 1ms
    digitalWrite(OTT2, HIGH);
    Serial.println("OK");
  }
}

// rotate motor
int Step(int steps, int pin)
{
  if (!active_h && pin == STP_h)
  {
    Serial.println("horizontal motor still deactivated");
  }
  if (!active_v && pin == STP_v)
  {
    Serial.println("vertical motor still deactivated");
  }
  
  delvar = 6000;
  for (x = 1; x <= steps; x++)  {
    if (button_pushed) {
      button_pushed = false;
      break;
    }
    digitalWrite(pin, HIGH);
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
  if (steps == (x-1)) Serial.println("OK");
  return (x-1);
}
