#include <EEPROM.h>



int FindMinTPS(int, int, int);
int FindMaxTPS(int, int, int);
unsigned int FindMinAPP(int);
unsigned int FindMaxAPP(int);
bool burnCalibration(int,int,int,int,int,int);
bool clearCalibration();

int FindMinTPS(int HB_PWM, int HB_DIRECTION, int POT_THROTTLE){
  digitalWrite(HB_DIRECTION, 1);
  analogWrite(HB_PWM, 90);
  
  delay(500);
  int MinTPSV = analogRead(POT_THROTTLE);
  MinTPSV = analogRead(POT_THROTTLE) - 10;
  Serial.println(MinTPSV);
  return MinTPSV;
  //closing
}


int FindMaxTPS(int HB_PWM, int HB_DIRECTION, int POT_THROTTLE){
  digitalWrite(HB_DIRECTION, 0);
  analogWrite(HB_PWM, 195);
  
  delay(500);
  int MaxTPSV = analogRead(POT_THROTTLE);
  MaxTPSV = analogRead(POT_THROTTLE);
  Serial.println(MaxTPSV);
  return MaxTPSV;
}

int FindRestingTPS(int HB_PWM, int HB_DIRECTION, int POT_THROTTLE){
    analogWrite(HB_PWM, 0);
  digitalWrite(HB_DIRECTION, 1);
  delay(700);
  int RestingTPSV = analogRead(POT_THROTTLE);
  RestingTPSV = analogRead(POT_THROTTLE);
  return RestingTPSV;
}

unsigned int FindMaxAPP(int POT_PEDAL){
  Serial.println("Please depress APP");
  delay(1000);
  unsigned int MaxAPP = analogRead(POT_PEDAL);
  MaxAPP = analogRead(POT_PEDAL);
  Serial.print("Max APP raw val: ");Serial.println(MaxAPP);
  delay(1000);
  return MaxAPP;
}

unsigned int FindMinAPP(int POT_PEDAL){
  Serial.println("Please let go of APP");
  delay(1000);
  unsigned int MinAPP = analogRead(POT_PEDAL);
  MinAPP = analogRead(POT_PEDAL);
  Serial.print("Min APP raw val: ");Serial.println(MinAPP);
  delay(1000);
  return MinAPP;
}

bool burnCalibration(int minTPS, int maxTPS, int minAPP, int maxAPP, int minTPS2, int maxTPS2, int minAPP2, int maxAPP2, int restTPS, int throttleMode, int thirdPointAPP, int thirdPointTPS, int secondAPP2, int secondTPS2, int secondOutAPP2, int secondOutTPS2, int thirdAPP2, int thirdTPS2, int thirdOutAPP2, int thirdOutTPS2){
  FastCRC8 CRC8;
  
  Serial.print("MaxTPS = "); Serial.println(maxTPS);
  Serial.print("MinTPS = "); Serial.println(minTPS);
  Serial.print("MaxAPP = "); Serial.println(maxAPP);
  Serial.print("MinAPP = "); Serial.println(minAPP);
  EEPROM.update(0,highByte(minTPS));
  EEPROM.update(1,lowByte(minTPS));
  EEPROM.update(2,highByte(maxTPS));
  EEPROM.update(3,lowByte(maxTPS));
  EEPROM.update(4,highByte(minAPP));
  EEPROM.update(5,lowByte(minAPP));
  EEPROM.update(6,highByte(maxAPP));
  EEPROM.update(7,lowByte(maxAPP));
  EEPROM.update(8,true);// address 8 saved calibration status
  EEPROM.update(9,highByte(restTPS));
  EEPROM.update(10,lowByte(restTPS));
  EEPROM.update(11,highByte(minTPS2));
  EEPROM.update(12,lowByte(minTPS2));
  EEPROM.update(13,highByte(maxTPS2));
  EEPROM.update(14,lowByte(maxTPS2));
  EEPROM.update(15,highByte(minAPP2));
  EEPROM.update(16,lowByte(minAPP2));
  EEPROM.update(17,highByte(maxAPP2));
  EEPROM.update(18,lowByte(maxAPP2));
  EEPROM.update(19,throttleMode);
  EEPROM.update(20,thirdPointAPP);
  EEPROM.update(21,thirdPointTPS);
  EEPROM.update(22,highByte(secondAPP2));
  EEPROM.update(23,lowByte(secondAPP2));
  EEPROM.update(24,highByte(secondOutAPP2));
  EEPROM.update(25,lowByte(secondOutAPP2));
  EEPROM.update(26,highByte(secondTPS2));
  EEPROM.update(27,lowByte(secondTPS2));
  EEPROM.update(28,highByte(secondOutTPS2));
  EEPROM.update(29,lowByte(secondOutTPS2));
  EEPROM.update(30,highByte(thirdAPP2));
  EEPROM.update(31,lowByte(thirdAPP2));
  EEPROM.update(32,highByte(thirdOutAPP2));
  EEPROM.update(33,lowByte(thirdOutAPP2));
  EEPROM.update(34,highByte(thirdTPS2));
  EEPROM.update(35,lowByte(thirdTPS2));
  EEPROM.update(36,highByte(thirdOutTPS2));
  EEPROM.update(37,lowByte(thirdOutTPS2));
  Serial.println("All calibration values are saved");

  uint8_t CRCbuf[37];
  for (int i = 0; i < 38; i++){
    CRCbuf[i] = EEPROM.read(i);
    Serial.print("CRC buffer[");Serial.print(i);Serial.print("] = ");Serial.println(CRCbuf[i]);
    Serial.print("EEPROM[");Serial.print(i);Serial.print("] = ");Serial.println(EEPROM.read(i));
  }
  EEPROM.update(38, CRC8.smbus(CRCbuf, sizeof(CRCbuf)));
  Serial.print("CRC value = "); Serial.println(CRC8.smbus(CRCbuf, sizeof(CRCbuf)));
  return true; //to be used to set calibration flag
}

bool clearCalibration(){
  EEPROM.update(0,highByte(0));
  EEPROM.update(1,lowByte(0));
  EEPROM.update(2,highByte(0));
  EEPROM.update(3,lowByte(0));
  EEPROM.update(4,highByte(0));
  EEPROM.update(5,lowByte(0));
  EEPROM.update(6,highByte(0));
  EEPROM.update(7,lowByte(0));
  
    EEPROM.update(8,false);// address 8 saved calibration status
    EEPROM.update(9,highByte(0));
  EEPROM.update(10,0);
  
  EEPROM.update(11, 0);
  EEPROM.update(12,0);
  EEPROM.update(13,0);
  EEPROM.update(14,0);
  EEPROM.update(15,0);
  EEPROM.update(16,0);
  EEPROM.update(17,0);
  EEPROM.update(18,0);
  EEPROM.update(19,0);
  EEPROM.update(20,5);
  EEPROM.update(21,5);
  EEPROM.update(22,0);
  EEPROM.update(23,0);
  EEPROM.update(24,0);
  EEPROM.update(25,0);
  EEPROM.update(26,0);
  EEPROM.update(27,0);
  EEPROM.update(28,0);
  EEPROM.update(29,0);
  EEPROM.update(30,0);
  EEPROM.update(31,0);
  EEPROM.update(32,0);
  EEPROM.update(33,0);
  EEPROM.update(34,0);
  EEPROM.update(35,0);
  EEPROM.update(36,0);
  EEPROM.update(37,0);
  Serial.println("Calibration Cleared");
  return false;
}


