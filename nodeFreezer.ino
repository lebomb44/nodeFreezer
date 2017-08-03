#include <Cmd.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RF24.h>
#include <LbMsg.h>
#include <ID.h>

#define RED_LIGHT_PIN 5    /* ERROR on NRF24 msg */
#define ORANGE_LIGHT_PIN 6 /* NRF24 msg received for nodeFreezer */
#define GREEN_LIGHT_PIN 7  /* Send temperature on NRF24 */

#define NRF24_CE_pin   9
#define NRF24_CSN_pin  10
#define NRF24_MOSI_pin 11
#define NRF24_MISO_pin 12
#define NRF24_SCK_pin  13

OneWire oneWire(2);
DallasTemperature tempSensors(&oneWire);
RF24 nrf24(NRF24_CE_pin, NRF24_CSN_pin);
uint16_t nrf24SendTempCycle = 0;
bool nrf24_printIsEnabled = true;

/* Lights */
void redON   (int arg_cnt, char **args) { digitalWrite(RED_LIGHT_PIN   , HIGH); Serial.println("Red light ON"    ); }
void redOFF   (int arg_cnt, char **args) { digitalWrite(RED_LIGHT_PIN   , LOW); Serial.println("Red light OFF"   ); }
void orangeON(int arg_cnt, char **args) { digitalWrite(ORANGE_LIGHT_PIN, HIGH); Serial.println("Orange light ON" ); }
void orangeOFF(int arg_cnt, char **args) { digitalWrite(ORANGE_LIGHT_PIN, LOW); Serial.println("Orange light OFF"); }
void greenON (int arg_cnt, char **args) { digitalWrite(GREEN_LIGHT_PIN , HIGH); Serial.println("Green light ON"  );  }
void greenOFF (int arg_cnt, char **args) { digitalWrite(GREEN_LIGHT_PIN , LOW); Serial.println("Green light OFF" ); }

/* NRF24 */
void nrf24EnablePrint(int arg_cnt, char **args) { nrf24_printIsEnabled = true; Serial.println("NRF24 print enabled"); }
void nrf24DisablePrint(int arg_cnt, char **args) { nrf24_printIsEnabled = false; Serial.println("NRF24 print disabled"); }

void nrf24SendTempTo(uint8_t dst) {
  /* Send temperature to central system */
  digitalWrite(GREEN_LIGHT_PIN, HIGH);
  tempSensors.requestTemperatures();
  float tempF = tempSensors.getTempCByIndex(0);
  //Serial.println(tempF);
  int16_t tempI = (10.0 * tempF);
  //Serial.println(tempI);
  LbMsg msg(sizeof(int16_t));
  msg.setSrc(ID_BOURDILOT_FRIDGE);
  msg.setDst(dst);
  msg.setCmd(ID_BOURDILOT_FRIDGE_TEMP_TM);
  msg.getData()[0] = 0x00FF & (tempI>>8);
  msg.getData()[1] = 0x00FF & (tempI);
  msg.compute();
  if(true == nrf24_printIsEnabled) { Serial.print("Sending Temperature on NRF24 to dst="); Serial.print(dst); Serial.print("...");}
  nrf24.stopListening();
  bool writeStatus = nrf24.write(msg.getFrame(), msg.getFrameLen());
  nrf24.startListening();
  if(true == nrf24_printIsEnabled) {
    if(true == writeStatus) { Serial.println("OK"); } else { Serial.println("ERROR"); }
    Serial.print("Temperature message sent: "); msg.print(); Serial.println();
  }
  digitalWrite(GREEN_LIGHT_PIN, LOW);
}

void nrf24SendTempToLOST(int arg_cnt, char **args) { nrf24SendTempTo(ID_LOST); }

void setup() {
  pinMode(RED_LIGHT_PIN, OUTPUT);
  pinMode(ORANGE_LIGHT_PIN, OUTPUT);
  pinMode(GREEN_LIGHT_PIN, OUTPUT);
  digitalWrite(RED_LIGHT_PIN, HIGH);
  digitalWrite(ORANGE_LIGHT_PIN, HIGH);
  digitalWrite(GREEN_LIGHT_PIN, HIGH);

  Serial.begin(115200);
  Serial.println("nodeFreezer Starting...");

  tempSensors.begin();

  Serial.print("NRF24 begin...");
  if(true == nrf24.begin()) { Serial.println("OK"); } else { Serial.println("ERROR"); }
  nrf24.setPALevel(RF24_PA_LOW);
  Serial.println("NRF24 PA level set");
  //nrf24.setAddressWidth(4);
  Serial.println("NRF24 address width set");
  uint8_t nrf24AdrR[6] = "FRIDG";
  nrf24.openReadingPipe(1, nrf24AdrR);
  Serial.println("NRF24 reading pipe opened");
  uint8_t nrf24AdrW[6] = "LOST+";
  nrf24.openWritingPipe(nrf24AdrW);
  Serial.println("NRF24 writing pipe opened");
  nrf24.startListening();
  Serial.println("NRF24 listening started");

  cmdInit();

/*
  cmdAdd("redON"   , "Red Light ON"   , redON   );  cmdAdd("redOFF"   , "Red Light OFF"   , redOFF   );
  cmdAdd("orangeON", "Orange Light ON", orangeON);  cmdAdd("orangeOFF", "Orange Light OFF", orangeOFF);
  cmdAdd("greenON" , "Green Light ON" , greenON );  cmdAdd("greenOFF" , "Green Light OFF" , greenOFF );
  cmdAdd("nrf24SendTempLOST", "Send temperature on NRF24 link", nrf24SendTempToLOST);
  cmdAdd("nrf24EnablePrint", "Enable print in NRF24 lib", nrf24EnablePrint);
  cmdAdd("nrf24DisbalePrint", "Disable print in NRF24 lib", nrf24DisablePrint);
  cmdAdd("help", "List commands", cmdList);
*/

  Serial.println("nodeFreeze Init done");

  delay(1000);
  digitalWrite(RED_LIGHT_PIN, LOW);
  digitalWrite(ORANGE_LIGHT_PIN, LOW);
  digitalWrite(GREEN_LIGHT_PIN, LOW);
  nrf24_printIsEnabled=true;

}

void loop() {
  delay(1);

  /* Read incoming messages */
  if(true == nrf24.available()) {
    LbMsg msg(32-4-1); /* 32 bytes max in NRF24 static payload */
    nrf24.read(msg.getFrame(), 4);
    nrf24.read(msg.getData(), min(msg.getDataLen()+1, 32-4));
    if(true == nrf24_printIsEnabled) { Serial.print("NRF24 rx: "); msg.print(); }
    if(true == msg.check()) {
      if(true == nrf24_printIsEnabled) { Serial.println(": OK"); }
      /* Actions to do */
      if(ID_BOURDILOT_FRIDGE == msg.getDst()) {
        digitalWrite(ORANGE_LIGHT_PIN, HIGH);
        if(ID_BOURDILOT_FRIDGE_NETWORK_TC == msg.getCmd()) {
          Serial.println("Network TC");
        }
        else if(ID_BOURDILOT_FRIDGE_TEMP_TC == msg.getCmd()) {
          nrf24SendTempTo(msg.getSrc());
        }
        else {
          digitalWrite(RED_LIGHT_PIN, HIGH);
          Serial.print("NRF24 cmd UNKNOWN : "); Serial.println(msg.getCmd());
          digitalWrite(RED_LIGHT_PIN, LOW);
        }
        digitalWrite(ORANGE_LIGHT_PIN, LOW);
      }
      else {
        if(true == nrf24_printIsEnabled) { Serial.print("NRF24 msg not for me dst="); Serial.println(msg.getDst()); }
      }
    }
    else {
      digitalWrite(RED_LIGHT_PIN, HIGH);
      if(true == nrf24_printIsEnabled) { Serial.println(": Bad CKS !"); }
      digitalWrite(RED_LIGHT_PIN, LOW);
    }
  }

  if(1000 < nrf24SendTempCycle) { nrf24SendTempTo(ID_LOST); nrf24SendTempCycle = 0; }
  nrf24SendTempCycle++;

  cmdPoll();
}

