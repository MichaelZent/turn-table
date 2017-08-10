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
//Other turntable
#define OTT1 0
#define OTT2 1
//horizontal motor
#define nSLP_h 4
#define STP_h 5
#define DIR_h 6
#define BUTTON 7
//vertical motor
#define nSLP_v 10
#define STP_v 11
#define DIR_v 12
#define REED 13

#define SENSOR A2 // analog input distance sensor
#define THRESHOLD_SENSOR 100

// memory addresses
#define EEPROM_ADDR_TRANS_h 0
#define EEPROM_ADDR_TRANS_v 1
#define EEPROM_ADDR_mSTEPS_h 2
#define EEPROM_ADDR_mSTEPS_v 3
#define EEPROM_ADDR_OFFSET_h 5
#define EEPROM_ADDR_OFFSET_v 6

// global variables
bool active_h, active_v, is_2D = false; // active_x: true if motor is activated ; is_2D: true if both directions are wanted
bool button_pushed = false; //set to true after button was pushed, reset to false immediatly
bool dir = true; // true if horizontal, false if vertical
//int x, y, val = 0;


int msteps_h, msteps_v; //motorsteps, loaded from EEPROM or set by user
float transmission_h, transmission_v; //transmission, loaded from EEPROM
signed int offset_h, offset_v;  // marker/reed offset
float spd_h, spd_v;  // steps-per-degree
int trig_step_width_h, trig_step_width_v; // triggering from other turntable, tbd
//current angle
float angle_h_is = 0;
float angle_v_is = 0;
//targeted angle
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
  attachInterrupt(digitalPinToInterrupt(BUTTON), button, FALLING); //interrupt when button pushed

  Serial.begin(9600); // Open Serial connection
  Wire.begin();
  
  //load from EEPROM
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
    offset_v = offset_tmp;
    Serial.println(offset_v);
  }
  //calculate spd
  spd_h = transmission_h * msteps_h / 1.8;
  trig_step_width_h = 10 * transmission_h;
  spd_v = msteps_v * transmission_v / 1.8;
  trig_step_width_v = 10 * transmission_v;

  Serial.println(spd_h);
  Serial.println(trig_step_width_h);
  Serial.println(spd_v);
  Serial.println(trig_step_width_v);

}

// ISR: set global variable, variable used for intercepts while in movements
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

  /*
  if (digitalRead(REED) == 1)
  {
    Serial.println("1");
    delay(500);
  }
  if (digitalRead(REED) == 0)
  {
    Serial.println("0");
    delay(500);
  }
*/
  //activate vertical motor by pushing button
  if (button_pushed) {
    button_pushed = false;
    Serial.write("Button pushed");
    if (!active_v) {
      active_v = true;
      digitalWrite(nSLP_v, HIGH);
    }
  }

  //balance();




  //activate horizontal motor when marker is moved over distance sensor
  if (!active_h && analogRead(SENSOR) < THRESHOLD_SENSOR) {
    active_h = true;
    Serial.println(F("horizontal motor activated"));
    digitalWrite(nSLP_h, HIGH); // set controller active
    digitalWrite(DIR_h, HIGH); // move clockwise
    delay(100);
    //Step((offset_h * spd_h), STP_h); // go to 0 degrees
  }


  
  if (Serial.available()) {
    parse();
  }
  else {
   int  val = digitalRead(OTT1);
    if (val == 0) {
      Step(trig_step_width_h, STP_h);
      delay(2); //make sure to not catch the same trigger again
    }
  }
}

// return to vertical equilibrium (offset_v), move up till Reed is LOW then move down slowly until Reed switches from LOW to HIGH (known offfset) then moves this offset to equilibrium
void balance_v()
{
  if (!active_v) {
    active_v = true;
    digitalWrite(nSLP_v, HIGH);
    Serial.println(F("Vertical motor activated"));
  }

  if (digitalRead(REED) == LOW) {

    digitalWrite(DIR_v, HIGH);
int    delvar = 6000;
    while (digitalRead(REED) == LOW) {

      
      digitalWrite(STP_v, HIGH);
      delayMicroseconds(delvar);
      digitalWrite(STP_v, LOW);
      delayMicroseconds(delvar);
          if (button_pushed) {
      Serial.println(F("Warning: Movement stopped by button. Angles might be incorrect now! Initialize again!"));
      button_pushed = false;
      return;
    }

      
    }

    digitalWrite(DIR_v, LOW);
    int steps = offset_v* spd_v;
    Step(steps, STP_v);

    angle_v_is = 0;
    angle_v_target = 0;
    Serial.println(F("Vertical equilibrium reached.  Angle_v_is = 0.00"));
  }
  else {

    digitalWrite(DIR_v, HIGH);
int    delvar = 6000;
    Serial.println(F("Adjusting vertical angle. If antenna is higher than 45 degree push the button!"));
    while ( digitalRead(REED) == HIGH) {


      digitalWrite(STP_v, HIGH);
      delayMicroseconds(delvar);
      digitalWrite(STP_v, LOW);
      delayMicroseconds(delvar);
          if (button_pushed) {
      button_pushed = false;
      Serial.println(F("Warning: Movement stopped by button. Angles might be incorrect now! Initialize again!"));
      return;
    }

    }
    balance_v();

  }
}

//turn to wanted position; slowly move clockwise till marker is over distance sensor, then move fast to position; Warning: turns fast and many degrees, look out for cable problems
void balance_h() {

  if (!active_h) {
    active_h = true;
    digitalWrite(nSLP_h, HIGH);
    Serial.println(F("Horizontal motor activated"));
  }

  digitalWrite(DIR_h, HIGH);
int  delvar = 4000;
  while (analogRead(SENSOR) > THRESHOLD_SENSOR) {

    digitalWrite(STP_h, HIGH);
    delayMicroseconds(delvar);
    digitalWrite(STP_h, LOW);
    delayMicroseconds(delvar);
        if (button_pushed) {
      button_pushed = false;
      Serial.println(F("Warning: Movement stopped by button. Angles might be incorrect now! Initialize again!"));
      break;
    }

  }

  if (analogRead(SENSOR) > THRESHOLD_SENSOR) {
    Serial.println(F("Warning: Marker not in right position, try again!"));
    return;
  }
  else {
    Step((offset_h * spd_h), STP_h); // go to 0 degrees
    angle_h_is = 0;
    angle_h_target = 0;
  Serial.println(F("Angle_h_is = 0.00"));
  }
}

//defines new equilibrium; Sets new 0 degree mark. Reed has to be LOW to do it succesfully. Drives down till Reed is high to set new offset. Warning: Box has to be in vertical equiibrium when used! If not, the 0 degree is set wrong and dangerous positions can be reached. 
void equilibrium() {

  angle_v_is = 0;
  angle_v_target = 0;

  if (digitalRead(REED) == LOW) {

    digitalWrite(DIR_v, HIGH);
    int  delvar = 6000;
    int offset_stp_v = 0;
    while (digitalRead(REED) == LOW) {

      digitalWrite(STP_v, HIGH);
      delayMicroseconds(delvar);
      digitalWrite(STP_v, LOW);
      delayMicroseconds(delvar);
          if (button_pushed) {
      Serial.println(F("Warning: Movement stopped by button. Angles might be incorrect now! Initialize again!"));
      button_pushed = false;
      break;
    }
      offset_stp_v ++;
    }

    offset_v = offset_stp_v / spd_v;

  }
  else {
    Serial.println(F("Error: In vertical equilibrium reed switch should be closed! Try moving the antenna down and back to equilibrium again and retry."));



  }
}

//moves a given angle in given direction
void move(float angle, bool dir) {
  int steps = 0;
    //Serial.print(F("Angle:  "));
    //Serial.println(angle);
  if(dir == true) {

    //limit the target angle between 0° and 360°
    if (is_2D && (angle_h_target > 360 || angle_h_target < 0)) {

      if (angle_h_target > 360) {

        angle_h_target = angle_h_target - 360;

      }

      if (angle_h_target < 0) {

        angle_h_target = angle_h_target + 360;
      }
    }

    //Serial.print(F("Target angle:  "));
    //Serial.println(angle_h_target);
    //Serial.print(F("Angle:  "));
    //Serial.println(angle);
    if (angle < 0) {
      angle = abs(angle);
      if (is_2D) digitalWrite(DIR_h, HIGH); // horizontal motors are flipped ??
      else digitalWrite(DIR_h, LOW);
      steps = angle * spd_h;
      angle_h_is -= Step(steps, STP_h) / spd_h;
      if (angle_h_is < 0) {
        angle_h_is += 360;
      }
      //Serial.print(F("New Angle_h: "));
      //Serial.println(angle_h_is);
    }
    else {
      if (is_2D) digitalWrite(DIR_h, LOW);
      else digitalWrite(DIR_h, HIGH);
      steps = angle * spd_h;
      angle_h_is += Step(steps, STP_h) / spd_h;

      if (angle_h_is >= 360) {

        angle_h_is -=  360;

      }

      //Serial.print(F("New Angle_h: "));
      //Serial.println(angle_h_is);
    }
    //Serial.print(F("Driven Steps_h: "));
    //Serial.println(steps);
  }
  else {
        if (angle < 0) {
        digitalWrite(DIR_v, LOW);
        //Serial.print(F("Angle:"));
        //Serial.println(angle);
        angle = abs(angle);
        //Serial.print(F("Abs_Angle:"));
        //Serial.println(angle);
        steps = angle * spd_v;
        //Serial.print(F("Steps:"));
        //Serial.println(steps);
        angle_v_is -= Step(steps, STP_v) / spd_v;
        //Serial.print(F("New Angle_v: "));
        //Serial.println(angle_v_is);
      }
      else {
        digitalWrite(DIR_v, HIGH);
        steps = angle * spd_v;
        angle_v_is += Step(steps, STP_v) / spd_v;
        //Serial.print(F("New Angle_v: "));
        //Serial.println(angle_v_is);
      }
      //Serial.print(F("Driven Steps_v: "));
      //Serial.println(steps);
  }
}

// execute command from user
void parse() {
  char serialBuffer[32] = {0};
  byte num = Serial.readBytesUntil('\n', serialBuffer, 32);
  if (num == 0) {
    return;
  }
  char* tok = strtok(serialBuffer, " \r");
  if (tok == 0) {
    return;
  }

  if (strcmp(tok, "help") == 0) {
    Serial.println(F("get trig_width : get steps per trigger input"));
    Serial.println(F("get spd : get steps per degree"));
    Serial.println(F("get trans : get transmission ratio"));
    Serial.println(F("set trans x: set transmission ratio"));
    Serial.println(F("set msteps x: set microSteps of motorcontroller"));
    Serial.println(F("balance: moves to and sets 0 degree, for _v: moves antenna down and afterwards to 0, make sure the antenna is near to equilibrium and not lower than 30 degree"));
    Serial.println(F("move x: move x degrees, positive value for _h turns clockwise, positive value for _v turns antenna down"));
    Serial.println(F("moveto x: move to x degree, first use balance!"));
    Serial.println(F("equilibrium: sets equilibrium, usually not to use! only if balance_v does not move to equilibrium!"));
    Serial.println(F("trig : trigger remote table"));
    Serial.println(F("activate: activate motor"));
    Serial.println(F("deactivate: deactivate motor"));
    Serial.println(F("sleep: deactivate all motors"));
    Serial.println(F("status: get current position"));
    Serial.println(F("get position: get one current position"));
    if (is_2D) {
      Serial.println(F("-------------------------"));
      Serial.println(F("use suffix '_h' for horizontal and '_v' for vertical control"));
      Serial.println(F("eg: 'move_v x' or 'set msteps_h x'"));
    }
  }
  else if (strcmp(tok, "sleep") == 0) {
    active_h = active_v = false;
    Serial.println(F("Motors deacitvated"));
    digitalWrite(nSLP_h, LOW);
    digitalWrite(nSLP_v, LOW);
  }
  else if (strcmp(tok, "activate_h") == 0 || strcmp(tok, "activate") == 0) {
    active_h = true;
    digitalWrite(nSLP_h, HIGH);
    Serial.println(F("horizontal motor activated"));
  }
  else if (strcmp(tok, "activate_v") == 0) {
    active_v = true;
    digitalWrite(nSLP_v, HIGH);
    Serial.println(F("vertical motor activated"));
  }
  else if (strcmp(tok, "deactivate_h") == 0 || strcmp(tok, "deactivate") == 0)  {
    active_h = false;
    digitalWrite(nSLP_h, LOW);
    Serial.println(F("horizontal motor deactivated"));
  }
  else if (strcmp(tok, "deactivate_v") == 0) {
    active_v = false;
    digitalWrite(nSLP_v, LOW);
    Serial.println(F("vertical motor deactivated"));
  }
  else if (strcmp(tok, "balance_v") == 0) {
    balance_v();
  }
  else if (strcmp(tok, "balance_h") == 0) {
    balance_h();
  }
  else if (strcmp(tok, "initialize") == 0) {
    balance_h();
    balance_v();
  }
  else if (strcmp(tok, "equilibrium_v") == 0 || strcmp(tok, "equilibrium") == 0) {
    
    equilibrium();
  }

  else if (strcmp(tok, "status") == 0) {
    Serial.println(angle_h_is);
    if (is_2D) {
      Serial.println(angle_v_is);
    }
  }  else if (strcmp(tok, "get") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      return;
    }
    else if (strcmp(tok2, "spd") == 0 || strcmp(tok2, "spd_h") == 0) {
      Serial.println(spd_h);
    }
    else if (strcmp(tok2, "spd_v") == 0) {
      Serial.println(spd_v);
    }
    else if (strcmp(tok2, "position") == 0 || strcmp(tok2, "position_h") == 0) {
      Serial.println(angle_h_is);
    }
    else if (strcmp(tok2, "position_v") == 0) {
      Serial.println(angle_v_is);
    }
    else if (strcmp(tok2, "trig_width") == 0 || strcmp(tok2, "trig_width_h") == 0) {
      Serial.println(trig_step_width_h);
    }
    else if (strcmp(tok2, "trig_width_v") == 0) {
      Serial.println(trig_step_width_v);
    }
    else if (strcmp(tok2, "trans") == 0 || strcmp(tok2, "trans_h") == 0) {
      Serial.println(transmission_h);
    }
    else if (strcmp(tok2, "trans_v") == 0) {
      Serial.println(transmission_v);
    }
    else if (strcmp(tok2, "msteps") == 0 || strcmp(tok2, "msteps_h") == 0) {
      Serial.println(msteps_h);
    }
    else if (strcmp(tok2, "msteps_v") == 0) {
      Serial.println(msteps_v);
    }
    else if (strcmp(tok2, "offset") == 0 || strcmp(tok2, "offset_h") == 0) {
      Serial.println(offset_h);
    }
    else if (strcmp(tok2, "offset_v") == 0) {
      Serial.println(offset_v);
    }
    else {
      Serial.print(F("Unknown command: "));
      Serial.println(tok2);
      Serial.println(F("Command 'Help' for help"));
      return;
    }
  }
  else if (strcmp(tok, "set") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      return;
    }
    else if (strcmp(tok2, "trans") == 0 || strcmp(tok2, "trans_h") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        Serial.println(F("Error: No value"));
        return;
      }
      char* next;
      transmission_h = strtod(tok3, &next);
      EEPROM.update(EEPROM_ADDR_TRANS_h, transmission_h * 10);
    }
    else if (strcmp(tok2, "trans_v") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        Serial.println(F("Error: No value"));
        return;
      }
      char* next;
      transmission_v = strtod(tok3, &next);
      EEPROM.update(EEPROM_ADDR_TRANS_v, transmission_v * 10);
    }
    else if (strcmp(tok2, "msteps") == 0 || strcmp(tok2, "msteps_h") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        Serial.println(F("Error: No value"));
        return;
      }
      char* next;
      msteps_h = strtol(tok3, &next, 10);
      EEPROM.update(EEPROM_ADDR_mSTEPS_h, msteps_h);
    }
    else if (strcmp(tok2, "msteps_v") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        Serial.println(F("Error: No value"));
        return;
      }
      char* next;
      msteps_v = strtol(tok3, &next, 10);
      EEPROM.update(EEPROM_ADDR_mSTEPS_v, msteps_v);
    }
    else if (strcmp(tok2, "offset_h") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        Serial.println(F("Error: No value"));
        return;
      }
      char* next;
      offset_h = strtol(tok3, &next, 10) ;
      EEPROM.update(EEPROM_ADDR_OFFSET_h, offset_h);
    }
    else if (strcmp(tok2, "offset_v") == 0) {
      char* tok3 = strtok(0, " \r");
      if (tok3 == 0) {
        Serial.println(F("Error: No value"));
        return;
      }
      char* next;
      offset_v = strtol(tok3, &next, 10) ;
      EEPROM.update(EEPROM_ADDR_OFFSET_v, offset_v);
    }
    else {
      Serial.print(F("Unknown command: "));
      Serial.println(tok2);
      Serial.println(F("Command 'Help' for help"));
      return;
    }

    Serial.println(F("OK"));
  }

  
  else if (strcmp(tok, "moveto") == 0 || strcmp(tok, "moveto_h") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      Serial.println(F("Error: No value"));
      return;
    }
    if (!active_h) {
      Serial.println(F("Error: Motor is deactivated"));
      return;
    }
    char* next;
    angle_h_target = strtod((const char*)tok2, &next);
    
    double angle = angle_h_target - angle_h_is;
    if(angle > 180) {
      angle = angle - 360;
    }
    
    dir = true;
    move(angle, dir);
  }
  
 else if (strcmp(tok, "moveto_v") == 0) {
 char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      Serial.println("Error: No value");
      return;
    }
    if (!active_v) {
      Serial.println("Error: Motor is deactivated");
      return;
    }
    char* next;
    int steps = 0;
   angle_v_target = strtod((const char*)tok2, &next);
       if (angle_v_target > 46 || angle_v_target < -46) {
      angle_v_target = angle_v_is;
      Serial.println("Error: Maximum target angle is +/-45 degree");
      return;
    }
    Serial.print(F("Target_Angle:"));
    Serial.println(angle_v_target);
   double angle = angle_v_target - angle_v_is;
    Serial.print(F("Angle:"));
    Serial.println(angle);   
   dir = false;
   move(angle,dir);
   
 }
  // limited between 0 and 360°
  else if (strcmp(tok, "move") == 0 || strcmp(tok, "move_h") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      Serial.println("Error: No value");
      return;
    }
    if (!active_h) {
      Serial.println("Error: Motor is deactivated");
      return;
    }
    char* next;
    int steps = 0;
    double angle = strtod((const char*)tok2, &next);
    if (angle > 360 || angle < -360) {
      Serial.println("Error: Rotation angle over 360 degree");
      return;
    }
    dir = true;
    angle_h_target += angle;
    move(angle, dir);
    
  }


  // limited between -46° and +46°
  else if (strcmp(tok, "move_v") == 0) {
    char* tok2 = strtok(0, " \r");
    if (tok2 == 0) {
      Serial.println("Error: No value");
      return;
    }
    if (!active_v) {
      Serial.println("Error: Motor is deactivated!");
      return;
    }
    char* next;
    double angle = strtod((const char*)tok2, &next);
    angle_v_target += angle;
    
    if (angle_v_target > 46 || angle_v_target < -46) {
      angle_v_target -= angle;
      Serial.println("Error: Maximum target angle is +/-45 degree");
      return;
    }
    dir = false;
     move(angle,dir);  
  }
  
  else if (strcmp(tok, "trig") == 0) {
    digitalWrite(OTT2, LOW);
    delay(1); // 1ms
    digitalWrite(OTT2, HIGH);
    Serial.println("OK");
  }

  else {
    Serial.print(F("Unknown command: "));
    Serial.println(tok);
    Serial.println(F("Command 'Help' for help"));
    return;

  }
}

// rotate motor
int Step(int steps, int pin)
{
  if (!active_h && pin == STP_h)
  {
    Serial.println(F("horizontal motor still deactivated"));
  }
  if (!active_v && pin == STP_v)
  {
    Serial.println(F("vertical motor still deactivated"));
  }

float  delvar = 6000;
int x= 1;
  for (x = 1; x <= steps; x++)  {
    if (button_pushed) {
      Serial.println(F("Warning: Movement stopped by button. Angles might be incorrect now! Initialize again!"));
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
  if (steps == (x - 1)) Serial.println(F("OK"));
  return (x - 1);
}
