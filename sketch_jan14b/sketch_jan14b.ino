#include <SoftwareSerial.h>
#include <Servo.h>

Servo actuatorX;
Servo actuatorY;

SoftwareSerial BLEtoSerial(2, 3); // RX, TX
#define  DIRECTION_X 'X'
#define DIRECTION_Y 'Y'
#define INDEX_STATE 0
#define INDEX_DIRECTION 2
#define INDEX_CONTROL_VALUE 4

#define SERVO_STEP  10  //DEG
#define WAIT_TIME 1000  //ms
#define ADC_AVERAGING_POINTS  5.0f
#define ADC_CHANNEL 5
#define ADC_VOLTS 5.0f
#define ADC_RESOLUTION  1023.0f

#define ERROR_FLOAT -1.0f
#define ERROR_INT -1

int adcArray[] = {0,1,2,3,4};

enum STATE
{
  STATE_IDLE = 0,
  STATE_MANUAL_RUNNING = 1,
  STATE_AUTO_RUNNING = 2,
};

STATE currentState = STATE_IDLE;
char* command;

void setup()
{
  // put your setup code here, to run once:
  actuatorX.attach(11);
  actuatorY.attach(10);
  
  BLEtoSerial.begin(115200);
  
  // for debug
  Serial.begin(9600);
}

void loop()
{
  // put your main code here, to run repeatedly:
  switch(currentState)
  {
    case STATE_IDLE:
      Command_Idle();
    break;
  
    case STATE_MANUAL_RUNNING:
      Serial.println("Manual Run Mode");
      Command_ManualRun();
    break;
    
    case STATE_AUTO_RUNNING:
      Serial.println("Auto Run Mode");
      Command_AutoRun();
    break;
  }
  
  delay(WAIT_TIME);
}

void Command_Idle()
{
  String s = SerialRead();
  char* rCommand = s.c_str();
  String c;
  
  if(rCommand[0] != '\0')
  {
    if(rCommand[INDEX_STATE] == '1')
    {
      currentState = STATE_AUTO_RUNNING;      
    }
    else if(rCommand[INDEX_STATE] == '2')
    {
      currentState = STATE_MANUAL_RUNNING;     
    }
    
    command = rCommand;
    c = rCommand;
    Serial.print("Recv Command : ");
    Serial.println(c);
  }
}

void Command_ManualRun()
{
  char directions = command[INDEX_DIRECTION];
  String sdeg = &command[INDEX_CONTROL_VALUE];
  float fdeg = StringToFloat(sdeg);
  
  //map(value, fromLow, fromHigh, toLow, toHigh) 
  //整数部のみ、少数は切り捨てられる
  //value: 変換したい数値 
  //fromLow: 現在の範囲の下限 
  //fromHigh: 現在の範囲の上限 
  //toLow: 変換後の範囲の下限 
  //toHigh: 変換後の範囲の上限 
  int decToPulse = map(fdeg,0,1023,0,179);

  if(directions == DIRECTION_X)
  {
    Serial.print("Write degX = ");
    Serial.println(fdeg);
    //req. conversion degToPulse
    actuatorX.write(fdeg);
  }
  else if(directions == DIRECTION_Y)
  {
    Serial.print("Write degY = ");
    Serial.println(fdeg);
    //req. conversion degToPulse
    actuatorY.write(fdeg);
  }
  else
  {
    Serial.print("conversion :");
    Serial.println(directions);
  }

  delay(1000);
  
  String responseWord = GetADCValue();

  SerialWrite(responseWord);

  currentState = STATE_IDLE;
}


void Command_AutoRun()
{
  for(int degX = 0; degX < 180; degX +=SERVO_STEP)
  {
    //req. conversion degToPulse
    
    Serial.print("Write degX = ");
    Serial.println(degX);
    
    actuatorX.write(degX);
    delay(WAIT_TIME);
    
    for(int degY = 0; degY < 80; degY +=SERVO_STEP)
    {
      Serial.print("Write degY = ");
      Serial.println(degY);
      
      //req. conversion degToPulse      
      actuatorY.write(degY);
      delay(WAIT_TIME);

      String s = SerialRead();
      char* rCommand = s.c_str();
      if(rCommand[0] == 'c')
      {
        currentState = STATE_IDLE;
        Serial.println("Cancel");
        return;
      }
      
      String responseWord = GetADCValue();
      
      Serial.print("response : ");
      Serial.println(responseWord);
      
      SerialWrite(responseWord);
    }
  }
}


String GetADCValue()
{
  int adc = 0;
  String responseWord;
  String sTmp;
  
  for(int iCh = 0; iCh < ADC_CHANNEL; iCh++)
  {   
    for(int cnt = 0; cnt < ADC_AVERAGING_POINTS; cnt++)
    {
      adc = analogRead(adcArray[iCh]);
    }
    
    sTmp = (adc/ADC_AVERAGING_POINTS) * ADC_VOLTS / ADC_RESOLUTION;
    Serial.print("Ch(ADC)[");
    Serial.print(iCh);
    Serial.print("] : value =");
    Serial.println(sTmp);
    
    responseWord += sTmp;
    if(iCh != ADC_CHANNEL-1)
    {
      responseWord += ",";
    }
  }
  
  return responseWord;
}


/*--------------*/
/*      Utility       */
/*--------------*/
float StringToFloat(String s)
{
  float result;
  char *ErrPtr;
  
  if(s=="") return ERROR_FLOAT;
  result=strtod(s.c_str(),&ErrPtr);
  if(*ErrPtr=='\0') {
    return result;
  } else {
    return ERROR_FLOAT;
  }
}

String SerialRead()
{
  String recvData = "";
  String abc = "";
  
  BLEtoSerial.listen();

  while(BLEtoSerial.available() > 0)
  {
    char rTmp = BLEtoSerial.read();
    recvData += rTmp;
  }

  return recvData;
}

void SerialWrite(String wword)
{
  BLEtoSerial.listen();

  delay(10);
  
  BLEtoSerial.println(wword);
}
