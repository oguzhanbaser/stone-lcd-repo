#include <SoftwareSerial.h>

SoftwareSerial displaySerial(2, 3);
uint16_t adcVal = 0;
int led1 = 13, led2 = 12, led3 = 11;
unsigned long lastTime = 0;

unsigned char arr[] = {
  0xA5, 0x5A, 0x05, 0x82, 0x19, 0x9A, 0x00, 0x20
};

unsigned char arrOn[] = {
  0xA5,0x5A,0x0C,0x82,0x1A,0x1B,0x4C,0x45,0x44,0x20,0x59,0x41,0x4E,0x44,0x49  
};

unsigned char arrOff[] = {
  0xA5,0x5A,0x0C,0x82,0x1A,0x1B,0x4C,0x45,0x44,0x20,0x53,0x4F,0x4E,0x44,0x55
};

uint8_t readSerialByte()
{
  while (displaySerial.available() < 1);
  uint8_t ret = displaySerial.read();
  return ret;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  displaySerial.begin(9600);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  Serial.println("Send At Commands");

}

void parseCMD(uint16_t mAdr, uint16_t mData)
{
  switch (mAdr)                  //check which button pressed with memory adress value
  {
    case 0x16A1:                //button1 adress
      digitalWrite(led2, !digitalRead(led2));
      break;
    case 0x16A2:                //button2 adress
      digitalWrite(led3, !digitalRead(led3));
      break;
    case 0x1992:                //switch adress
      digitalWrite(led1, mData);
      if(mData)                 //send string value according to switch status
      {
        displaySerial.write(arrOn, 15);       //send array on string value
      }else{
        displaySerial.write(arrOff, 15);      //send array off string value
      }
      break;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (displaySerial.available())          //check Serial Port buffer
  {
    if (readSerialByte() == 0xA5)         //Check first data is 0xA5
    {
      if (readSerialByte() == 0x5A)       //Check second data is 0x5A
      {
        uint8_t dataLen = readSerialByte();       //Read packetlen from first byte
        uint8_t sendCmd = readSerialByte();       //read cmd from second byte
        uint16_t dataAdr = (readSerialByte() << 8) |        //read and merge 16 bit data from buffer
                           readSerialByte();

        uint8_t valLen = readSerialByte();        //dataLen get
        uint16_t mVal = 0;
        for (int i = 0; i < valLen * 2; i++)      //get all data from buffer with loop
        {
          mVal = mVal << 8;
          mVal |= readSerialByte();
        }

        parseCMD(dataAdr, mVal);                //toggle LED with incoming data
        Serial.print(dataLen);                  //debug values
        Serial.print(" ");
        Serial.print(dataAdr, HEX);
        Serial.print(" ");
        Serial.println(mVal);
      }
    }

  }

  if (millis() - lastTime > 100)            //send data with 100 ms interval
  {
    adcVal = analogRead(A0);                  //read adc value
    adcVal = map(adcVal, 0, 1023, 0, 380);    //change ADC value limit to 0-380
    uint8_t sendValL = adcVal & 0xFF;         //get adc value LSB
    uint8_t sendValM = (adcVal & 0xFF00) >> 8;    //get ADC value MSB
    arr[6] = sendValM;
    arr[7] = sendValL;
    displaySerial.write(arr, 8);            //send adc value with Serial port
    lastTime = millis();
  }
}