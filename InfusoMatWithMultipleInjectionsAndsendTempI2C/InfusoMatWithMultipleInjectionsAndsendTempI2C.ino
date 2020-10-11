#include <I2C_Anything.h>

#include <timer.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <AltSoftSerial.h>
#include <Wire.h>
 
AltSoftSerial altSerial;

#define ONE_WIRE_BUS_1 6
#define ONE_WIRE_BUS_2 7

#define WIRE_STEPS_DEVICE 4
#define TEMPERATURE_DEVICE 8

#define DOSE_STEPS_RATIO 75


OneWire oneWire_in(ONE_WIRE_BUS_1); 
OneWire oneWire_out(ONE_WIRE_BUS_2);

DallasTemperature sensor_one(&oneWire_in);
DallasTemperature sensor_two(&oneWire_out);


const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int selectScreen = 0;
boolean screenCleared = false;
boolean firstDrawStart = true;
boolean start = false;
boolean stopProcedure = false;

boolean encButtonState = false;
boolean oldBtnState = false;
boolean switchFF = false;

boolean commandSet = false;
boolean errorMessage = false;
boolean okMessage = false;
boolean completeMessage = false;

boolean sendParams = false;
boolean waitForAnswer = false;
boolean calibrationComplete = false;

boolean holdInject = false;

boolean performingInjection = false;


boolean newCommand = false;

float infusionSpeed = 3;
float zeroOneStep = 0.1;
float threshold = 3;
float dose = 3;

float temp1;
float temp2;
float summedDifferenceT = 0;
float differenceT1 = 0;
float differenceT2 = 0;
float calibratedT1 = 0;
float calibratedT2 = 0;
float calibratedDiff = 0;

unsigned long lastTempTime;
unsigned long lastSendParamsTime;

int tempDelay = 1500;

unsigned long currentEncButtonTime;
unsigned long lastEncButtonTime = 0;
int encButtonDelay = 20;

//unsigned long lastIjectionTime = 0;
unsigned long injectionDelay = 300000;

long newEnc;
long oldEnc  = -999;
volatile int enc;

String dataString = "stop";
String answer = "";
String tempString = "";
int FB = 0;
int FBcounter = 0;
int incoming = 0;

int stepCounter = 0;

unsigned long infusionDelayCorrectionMinutes = 0;
float secondsIntermediateValue = 0;
unsigned long infusionDelayCorrectionSeconds = 0;
unsigned long infusionDelayCorrection = 0;

int infusionDelay = 0;
int delayCounter = 0;

void setup() {
  
  PCICR=1<<PCIE1; //разрешить прерывание 
  PCMSK1=(1<<PCINT9)|(1<<PCINT8); //выбрать входы A0 и A1
  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);
  pinMode(A2,INPUT); //feedback

//  pinMode(10, INPUT_PULLUP);
  
  Wire.begin();
  sensor_one.begin();
  sensor_one.setResolution(9);
  sensor_two.begin();
  sensor_two.setResolution(9);
  sensor_one.requestTemperatures();
  sensor_two.requestTemperatures();  
  temp1 = sensor_one.getTempCByIndex(0);
  temp2 = sensor_two.getTempCByIndex(0);

  Serial.begin(19200);
  altSerial.begin(4800);
  
  lcd.begin(16, 2);
  lcd.clear();
  analogWrite(A3, 128);
  
}


ISR (PCINT1_vect){
static byte old_n=PINC&3; // маска B00000011 что б читать только нужные 2 бита
byte new_n=PINC&3;
if (old_n==1&&new_n==3||old_n==2&&new_n==0) {PINC&4? enc++ : enc+=100 ;}
if (old_n==2&&new_n==3||old_n==1&&new_n==0) {PINC&4? enc-- : enc-=100 ;}
old_n= new_n;
}


void getEncValue(){
    newEnc = enc;
  if (newEnc != oldEnc) {
    oldEnc = newEnc;
  }
  
  encButtonState = digitalRead(10);
  if(encButtonState){    
    if(millis() - lastEncButtonTime > encButtonDelay){
      lastEncButtonTime = millis();
      switchFF = true;
    }
  }

}

void stopMotor(){
  dataString = "stop";
  altSerial.println(dataString);
  
}

void injectionDelayCounter(){
  if(holdInject){
    if(millis() - lastSendParamsTime >= injectionDelay){
      sendParams = true;
      injectionDelay=300000; 
      holdInject = false; 
    }
  }
}

void clrScreen(){
  if (screenCleared == false){
  lcd.clear();
  screenCleared = true; 
  switchFF = false; 
  }
}

void nullValues(){
  infusionSpeed = 3;
  threshold = 3;
  dose = 3;
  infusionDelay = 0;
  delayCounter = 0;
  summedDifferenceT = 0;
  differenceT1 = 0;
  differenceT2 = 0;
  calibratedT1 = 0;
  calibratedT2 = 0;
  calibrationComplete = false;
  injectionDelay = 300000;
}

void sendTemp(){
  if(millis() - lastTempTime >= tempDelay){ 
    lastTempTime = millis();
//    altSerial.println("initialisation");
//    altSerial.println("\n");
    tempString = "speed";  
    tempString += "temp1";
    tempString += temp1;
    tempString += "temp2";
    tempString += temp2;
    tempString += "end"; 
    altSerial.println(tempString);
    Serial.println(tempString);
  }
}


void feedback(){
  
 if(altSerial.available()){
  answer = altSerial.readStringUntil("\n");
//  int beginningOfMessage = answer.indexOf("commandRecieved");
//  answer.remove(0,beginningOfMessage);
  
 if(answer == "commandRecieved"){
  Serial.println(answer);
 }
 }
 }

void lowRes(){
  sensor_one.setResolution(9);
  sensor_two.setResolution(9);
}

void highRes(){
  sensor_one.setResolution(12);
  sensor_two.setResolution(12);
}

void errorCheck(){
  
  lowRes();
  
  if (temp2<=0 || temp1<=0){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sensor error!");
    stopMotor();
    delay(1000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Check cable or temp");
    lcd.setCursor(0, 1);
    lcd.print("is out of range!");
    stopMotor();
    delay(3000);
    lcd.clear();
    
  }
  if (temp2>=127 || temp1>=127){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sensor error!");
    stopMotor();
    delay(3000);
    lcd.clear();
  }  
}

void medicineDoseScreen(){
    clrScreen();
    
    lowRes();
    lcd.setCursor(0, 0);
    lcd.print("Set dose");
    lcd.setCursor(0, 1);
    lcd.print("(ml):");
    lcd.setCursor(5, 1);
    lcd.print(dose); 
    
    if (newEnc<enc){
      dose = dose + zeroOneStep;
      if(dose>5){
        dose = 5;
      }
      lcd.setCursor(5, 1);
      lcd.print("  ");
      lcd.setCursor(5, 1);
      lcd.print(dose);      
    }
    if (newEnc>enc){
      dose = dose - zeroOneStep;
      if(dose<3){
        dose = 3;
      }
      lcd.setCursor(5, 1);
      lcd.print("  ");
      lcd.setCursor(5, 1);
      lcd.print(dose);
    }

    if(encButtonState == false){
    if(switchFF){
      selectScreen = 1;
      screenCleared = false;
    }
    }
}

void timeScreen(){
    clrScreen();
    
    lowRes();
    lcd.setCursor(0, 0);
    lcd.print("Set time");
    lcd.setCursor(0, 1);
    lcd.print(" min:sec");
    lcd.setCursor(10, 1);
    lcd.print(infusionSpeed);
    if (newEnc<enc){
    infusionSpeed = infusionSpeed+zeroOneStep;
    if(infusionSpeed - int(infusionSpeed) > 0.50){
    infusionSpeed = infusionSpeed +1;
    infusionSpeed = int(infusionSpeed);
    }
    if(infusionSpeed>5){
    infusionSpeed = 5;
    }
      lcd.setCursor(10, 1);
      lcd.print("      ");
      lcd.setCursor(10, 1);
      lcd.print(infusionSpeed);      
    }
    if (newEnc>enc){
    infusionSpeed = infusionSpeed-zeroOneStep;
    if(infusionSpeed - int(infusionSpeed) > 0.50){
    infusionSpeed = infusionSpeed -0.5;
    infusionSpeed = int(infusionSpeed) + 0.50;
    }
    if(infusionSpeed<3){
    infusionSpeed = 3;
    }
      lcd.setCursor(10, 1);
      lcd.print("      ");
      lcd.setCursor(10, 1);
      lcd.print(infusionSpeed);
    }
    
    if(encButtonState == false){
    if(switchFF){
      selectScreen = 2;
      screenCleared = false;
    }
    }
}

void exceedingThresholdScreen(){
    clrScreen();
    
    lowRes();
    lcd.setCursor(0, 0);
    lcd.print("Set exceeding of");
    lcd.setCursor(0, 1);
    lcd.print("threshold:");
    lcd.setCursor(10, 1);   
    lcd.print(threshold);
    lcd.setCursor(15, 1);
    lcd.print("C");
 
    if (newEnc<enc){
      threshold = threshold+zeroOneStep;
      if(threshold>3){
        threshold = 3;
      }
      lcd.setCursor(10, 1);
      lcd.print("     C");
      lcd.setCursor(10, 1);
      lcd.print(threshold);      
    }
    if (newEnc>enc){
      threshold = threshold-zeroOneStep;
      if(threshold<0.1){
        threshold = 0.1;
      }
      lcd.setCursor(10, 1);
      lcd.print("     C");
      lcd.setCursor(10, 1);
      lcd.print(threshold);
    }

    if(encButtonState == false){
    if(switchFF){
      selectScreen = 3;
      screenCleared = false;
    }
    }
}



void confirmationScreen(){
    clrScreen();
    
    lowRes();
    lcd.setCursor(0, 0);
    lcd.print("Check:");
    lcd.setCursor(8, 0);    
    lcd.print("s:");
    lcd.setCursor(10, 0);
    lcd.print(infusionSpeed);
    lcd.setCursor(0, 1);
    lcd.print("th:");
    lcd.setCursor(3, 1);
    lcd.print(threshold);
    lcd.setCursor(8, 1);
    lcd.print("d:");
    lcd.setCursor(10, 1);
    lcd.print(dose);

    if(encButtonState == false){
      if(switchFF){
        selectScreen = 4;
        screenCleared = false;
      }
    }
}

void startScreen(){
    clrScreen();
    lowRes();
    if(firstDrawStart){
    lcd.setCursor(0, 0);
    lcd.print("Confirm start:");
    lcd.setCursor(0, 1);  
    lcd.print(">Back!<  Start! ");
    firstDrawStart = false;
    }

    if (newEnc<enc){
    start = true;
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(6, 1);
    lcd.print(" ");
    lcd.setCursor(8, 1);
    lcd.print(">");
    lcd.setCursor(15, 1);
    lcd.print("<");
    }
    if (newEnc>enc){
    start = false;
    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.setCursor(6, 1);
    lcd.print("<");
    lcd.setCursor(8, 1);
    lcd.print(" ");
    lcd.setCursor(15, 1);
    lcd.print(" ");
    }
    if(encButtonState == false){
      if(switchFF && start == false){
        screenCleared = false;
        firstDrawStart = true;
        start = false;
        selectScreen = 0;
      }
      if(switchFF && start){
        screenCleared = false;
        firstDrawStart = true;
        start = false;
        sendParams = true;
        selectScreen = 5;
      }
    }
}

void stopScreen(){
    clrScreen();
    lowRes();
    if(firstDrawStart){
    lcd.setCursor(0, 0);
    lcd.print("Cancel procedure?");
    lcd.setCursor(0, 1);  
    lcd.print(">Back!< Cancel! ");
    firstDrawStart = false;
    }

    if (newEnc<enc){
      stopProcedure = true;
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(6, 1);
      lcd.print(" ");
      lcd.setCursor(7, 1);
      lcd.print(">");
      lcd.setCursor(15, 1);
      lcd.print("<");
    }
    if (newEnc>enc){
      stopProcedure = false;
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(6, 1);
      lcd.print("<");
      lcd.setCursor(7, 1);
      lcd.print(" ");
      lcd.setCursor(15, 1);
      lcd.print(" ");
    }
    if(encButtonState == false){
    if(switchFF){
      if(!stopProcedure){
        screenCleared = false;
        firstDrawStart = true;
        selectScreen = 6;
      }
      if(stopProcedure){
        screenCleared = false;
        firstDrawStart = true;
        nullValues();
        
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Procedure");    
        lcd.setCursor(3, 1);
        lcd.print("Cancelled!");
        stopMotor();
        delay(1000);
        selectScreen = 0;
        stopProcedure = false;
      }
    }
    }
}

int calibrationStartTimestamp = 0;
int elapsed;
void calibrationScreen(){
  const int calibrationDurationMillis = 10000;
  clrScreen();
  highRes();
  
  if (calibrationStartTimestamp == 0){
    calibrationStartTimestamp = millis();
  }
  elapsed = millis() - calibrationStartTimestamp;
  
  if(elapsed >= calibrationDurationMillis){
    calibrationComplete = true;
    calibrationStartTimestamp = 0;
    selectScreen=6;
    screenCleared = false;
    calibratedT1 = temp1;
    calibratedT2 = temp2;
  }
  lcd.setCursor(0, 0);
  lcd.print("T1:");  
  lcd.print(temp1);
  lcd.setCursor(0, 1);
  lcd.print("T2:");
  lcd.print(temp2);
  lcd.setCursor(9, 0);
  lcd.print("SetTemp");
  lcd.setCursor(9, 1);
  lcd.print(elapsed / 1000);
}

float absDiff(float a, float b){
  if (a > b){
    return a-b;
  }
  return b-a;
}

void checkAlarm(){
  if((summedDifferenceT)>5){
       analogWrite(A3, 0);
       delay(15);
       analogWrite(A3, 128);
       delay(15);
       analogWrite(A3, 0);
       delay(15);
       analogWrite(A3, 128);
       delay(15);
       analogWrite(A3, 0);
       delay(15);
       analogWrite(A3, 128);
       delay(15);
  }
}

int coolDownLeft;
int lastProcScreen;
void procedureScreen2(){
  if (lastProcScreen == 1){
    lcd.clear();
  }
  
  lastProcScreen = 2;
  
  lcd.setCursor(0, 0);
  lcd.print("Th:");  
  lcd.print(threshold);
  lcd.print("sC:");
  lcd.print(stepCounter);
  lcd.setCursor(0, 1);
  lcd.print("TCool:");
  lcd.print(coolDownLeft / 1000);
}

void procedureScreen1(){
  if (lastProcScreen == 2){
    lcd.clear();
  }
  lastProcScreen = 1;
  
  lcd.setCursor(0, 0);
  lcd.print("t:");  
  lcd.print(temp1);
  lcd.print("|");
  lcd.print(temp2);
  lcd.setCursor(0, 1);
  lcd.print("D:");
  lcd.print(differenceT1);
  lcd.print("D2:");
  lcd.print(differenceT2);
}

void procedureFinishedScreen(){
  clrScreen();
  highRes();  
  lcd.setCursor(0, 0);
  lcd.print("Procedure finished"); 
  delay(1000);
  selectScreen = 6;
  screenCleared = false;
}

const int procedureScreenSwitchRate = 5000;
void procedureScreen(){
  clrScreen();
  highRes();
  if(switchFF){
    selectScreen = 7;
    screenCleared = false;
  }
  evaluateTDifference();
  checkAlarm();
   
  if ((millis() / procedureScreenSwitchRate) % 2 == 0) {
    procedureScreen1();
  }else{
    procedureScreen2();
  }

  if (performingInjection && dose * DOSE_STEPS_RATIO - stepCounter < 2){ 
     selectScreen = 8;
     performingInjection = false;
     screenCleared = false;
  } 
}

const int coolDown = 300000;
void sendParamsOnce(){
  String incoming = "";
//  FB = analogRead(A2);
//  Serial.println(FB);
  if (coolDownLeft > 0) {
    return;
  }
  dataString = "speed";
  dataString += infusionSpeed;
  dataString += "threshold";
  dataString += threshold;
  dataString += "dose";
  dataString += dose;
  dataString += "end";   
  altSerial.println(dataString);
  
  Serial.println(dataString);
  
  infusionDelayCorrectionMinutes = int(infusionSpeed)*60000;  

  secondsIntermediateValue = (infusionSpeed - int(infusionSpeed));

  infusionDelayCorrectionSeconds = secondsIntermediateValue*100000;
  infusionDelayCorrectionSeconds = infusionDelayCorrectionSeconds+1;

  infusionDelayCorrection = infusionDelayCorrectionMinutes + infusionDelayCorrectionSeconds;
  injectionDelay = injectionDelay+infusionDelayCorrection;
  sendParams = false;
  lastSendParamsTime = millis();
  holdInject = true;
  performingInjection = true;
  
  coolDownLeft = coolDown;
}


int refreshCoolDownTimestamp;
void refreshCoolDown(){
  const int rate = 1000;
  if (!timeHasCome(rate, &refreshCoolDownTimestamp)){
    return;
  }
  if (coolDownLeft > 0){
    coolDownLeft -= (millis() - lastSendParamsTime);
  }else{
    coolDownLeft = 0;
  }
}

bool timeHasCome(int rate, int* lastTime){
  int elapsed; 
  elapsed = millis() - *lastTime;
  if (elapsed > rate){
    *lastTime = millis();
    return true;  
  }
  return false; 
}

int getI2CDataStartTimestamp;
void getStepCount(){
  const int rate = 500;
  if (!timeHasCome(rate, &getI2CDataStartTimestamp)){
    return;
  }
 
  int newStepCount = 0; 
  int datalen = (sizeof newStepCount); 
  
  Wire.requestFrom(WIRE_STEPS_DEVICE, datalen, true); 
  if(Wire.available()==datalen){ // got correct size of packet
     I2C_readAnything(newStepCount);
  }

  stepCounter = newStepCount; 
}


void getTemp(){
  const int rate = 500;
  if (!timeHasCome(rate, &getI2CDataStartTimestamp)){
    return;
  }
  
  int datalen = (sizeof temp1)+(sizeof temp2); //вычисление кол-ва данных которые надо считать
  Wire.requestFrom(TEMPERATURE_DEVICE,datalen,true);
  if(Wire.available()==datalen){ // got correct size of packet
    I2C_readAnything (temp1);
    I2C_readAnything (temp2);
  }
 
}
 
volatile int getDebugTimestamp;
void debug(){
  const int rate = 1000;
  if (!timeHasCome(rate, &getDebugTimestamp)){
    return;
  }
  
  String debug;
  debug = "DEBUG calibrationComplete:"; debug += calibrationComplete;
//  debug += " summedDifferenceT:"; debug += summedDifferenceT;
//  debug += " threshold:";  debug += threshold;
  debug += " sendParams:";  debug += sendParams;
  debug += " coolDownLeft:";  debug += coolDownLeft;
  debug += " holdInject:"; debug += holdInject;
  debug += " injectionDelay:";  debug += injectionDelay;
  debug += " stepCounter:";  debug += stepCounter;
  debug += " steps left:";  debug += dose * DOSE_STEPS_RATIO - stepCounter;
  
  Serial.println(debug);
}

void evaluateTDifference(){
  differenceT1 = absDiff(temp1,calibratedT1);
  differenceT2 = absDiff(temp2,calibratedT2);
  
  summedDifferenceT = differenceT1+differenceT2;
}

void loop() {
  refreshCoolDown();
  getTemp();
  sendTemp();
  getEncValue();
  errorCheck();
  injectionDelayCounter();
  debug();
  if (calibrationComplete && (summedDifferenceT > threshold) && sendParams) {                  //startInjection Trigger 
      sendParamsOnce();
  }
  
  switch (selectScreen){
    case 0:
      medicineDoseScreen();
      break;
    case 1:
      timeScreen();
      break;
    case 2:
      exceedingThresholdScreen();
      break;  
    case 3:
      confirmationScreen();
      break;
    case 4:
      startScreen();
      break;  
    case 5:
      calibrationScreen();
      break;
    case 6:
      procedureScreen();
      break;
    case 7:
      stopScreen();
      break;
    case 8:
      procedureFinishedScreen();
      break;
  }
}
