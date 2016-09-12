String Version = "testSerialEsp8266Gateway";
#define debugConnection true
//-- comunication --
#include <SerialNetworkVariable.h>   // needed for communication with esp8266 gateway
#include <SerialNetworkVoid.h>  // needed for communication with esp8266 gateway
uint8_t PendingReqRefSerial = 0x01; // 0x01 request to the gateway means route (ready to eventualy add some more action in the gateway)
uint8_t PendingSecReqRefSerial = 0x01; // 0x01 request to the gateway means route
byte cycleRetrySendUnit = 0; // cycle retry check unitary command - used in case of acknowledgement needed from the Linux server
uint8_t trameNumber = 0;     // frame number to send
uint8_t lastAckTrameNumber = 0;  // last frame number acknowledged by the server
uint8_t pendingAckSerial = 0;    // flag waiting for acknowledged
int retryCount = 0;            // number of retry for sending
unsigned long timeSendSecSerial;  // used to check for acknowledgment of secured frames
unsigned long timeReceiveSerial;  // used to regularly check for received message
uint8_t diagConnection = 0x01;   // bit 0 pending serial link
unsigned long timeSendInfo;     // to regurarly send information to th server
#define delayBetweenInfo 5000   // delay before sending new status to the server  
uint8_t sendInfoSwitch = 0x00;  // flag to switch between different data to send
unsigned int count = 0;
#define esp8226ReadyPin 7        // to connect to ESP8266 ready PIN
void setup() {
  // put your setup code here, to run once:  Serial.begin(38400);            // for debugging log
  Serial2.begin(SpeedNetwSerial); // to communicate with the server through a serial Udp gateway
  pinMode(esp8226ReadyPin, INPUT);
  Serial.println(Version);
  Serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(esp8226ReadyPin) != 1)
  {
    Serial.println("wait for esp ready");
    delay(1000);
  }
  else {
    // ***  keep in touch with the server
    int getSerial = Serial_have_message();  // check if we have received a message
    if (getSerial > 0)                      // we got a message
    {
      TraitInput(DataInSerial[2]);          // analyze the message
    }
    if (PendingReqSerial != 0x00 )           // check if we have a message to send
    {
#if defined(debugConnection)
      Serial.println("send");
#endif
      DataToSendSerial();                    // send message on the serial link
      timeSendSecSerial = millis();          // reset timer
    }
    if (retryCount >= 5)                    // check retry overflow
    {
      pendingAckSerial = 0x00;               // clear retry flag
      retryCount = 0;                       // clear retry count
    }
    if ( millis() - timeSendSecSerial >= 5000 && pendingAckSerial != 0x00 && retryCount < 5) {
      ReSendSecuSerial();                    // re-send data that was not already ack by the server
#if defined(debugConnection)
      Serial.println("retry");
#endif
      timeSendSecSerial = millis();         // clear timer
      retryCount = retryCount + 1;          // update rerty count
    }
    if (retryCount >= 5)
    {
      retryCount = 0;
      pendingAckSerial = 0x00;
    }
    if (millis() - timeReceiveSerial >= 30000)     // 30 seconds without receiving data from the server
    {
      bitWrite(diagConnection, 1, 0);       // conection broken
    }

    if (millis() - timeSendInfo >= delayBetweenInfo )  // alternatively send status and power to the server
    {
      if (sendInfoSwitch % 2 == 0)
      {
        Send1();                 // send robot status to the server
      }
      if (sendInfoSwitch % 2 == 1)
      {
        Send2();             // send power info to the server
        count++;
      }
      sendInfoSwitch = sendInfoSwitch + 1;
      timeSendInfo = millis();
    }
    // *** end loop keep in touch with the server
  }
}
void Send1()
{
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x70; //
  PendingDataReqSerial[1] = 0x00; //
  PendingDataReqSerial[2] = 0x01;
  PendingDataReqSerial[3] = 0x00;
  PendingDataReqSerial[4] = 0x02;
  PendingDataReqSerial[5] = 0x00;
  PendingDataLenSerial = 0x06; // 6 longueur mini max 30 pour la gateway
}
void Send2()
{
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x70; //
  PendingDataReqSerial[1] = 0x00; //
  PendingDataReqSerial[2] = 0x03;
  PendingDataReqSerial[3] = 0x7f;
  PendingDataReqSerial[4] = 0x7e;
  PendingDataReqSerial[5] = 0x01;
  PendingDataReqSerial[6] = 0x01;
  PendingDataReqSerial[7] = uint8_t(count / 256);
  PendingDataReqSerial[8] = uint8_t(count);
  PendingDataReqSerial[29] = 0x02;
  PendingDataLenSerial = 0x1e; // 6 longueur mini max 30 pour la gateway

}
void TraitInput(uint8_t cmdInput) {
  Serial.print(".");
  int len = DataInSerial[3];
  for (int i = 0; i < len; i++) {
    Serial.print(DataInSerial[4 + i], HEX);
    Serial.print("-");
  }
  Serial.println();
}


