#include <Cmd.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RF24.h>
#include <LbMsg.h>
#include <ID.h>

/* *****************************
 *  Pin allocation
 * *****************************
 */
#define RED_LIGHT_PIN 5    /* ERROR on NRF24 msg */
#define ORANGE_LIGHT_PIN 6 /* NRF24 msg received for nodeFreezer */
#define GREEN_LIGHT_PIN 7  /* Send temperature on NRF24 */

#define NRF24_CE_pin   9
#define NRF24_CSN_pin  10
#define NRF24_MOSI_pin 11
#define NRF24_MISO_pin 12
#define NRF24_SCK_pin  13

/* *****************************
 *  Global variables
 * *****************************
 */
OneWire oneWire(2);
DallasTemperature tempSensors(&oneWire);
RF24 nrf24(NRF24_CE_pin, NRF24_CSN_pin);
uint16_t nrf24SendTempCycle = 0;

/* *****************************
 *  Debug Macros
 * *****************************
 */
bool nrf24_printIsEnabled = true;
#define NRF24_PRINT(m) if(true == nrf24_printIsEnabled) { m }

/* *****************************
 *  Command lines funstions
 * *****************************
 */
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
  msg.setSrc(ID_BOURDILOT_FREEZER_SLAVE);
  msg.setDst(dst);
  msg.setCmd(ID_BOURDILOT_FREEZER_TEMP_TM);
  msg.getData()[0] = 0x00FF & (tempI>>8);
  msg.getData()[1] = 0x00FF & (tempI);
  msg.compute();
  NRF24_PRINT( Serial.print("Sending Temperature on NRF24 to dst="); Serial.print(dst); Serial.print("..."); )
  nrf24.stopListening();
  bool writeStatus = nrf24.write(msg.getFrame(), msg.getFrameLen());
  nrf24.startListening();
  NRF24_PRINT(
    if(true == writeStatus) { Serial.println("OK"); } else { Serial.println("ERROR"); }
    Serial.print("Temperature message sent: "); msg.print(); Serial.println();
  )
  digitalWrite(GREEN_LIGHT_PIN, LOW);
}

void nrf24SendTempToLOST(int arg_cnt, char **args) { nrf24SendTempTo(ID_LOST_MASTER); }

void nrf24SendNetworkTo(uint8_t dst) {
  /* Send temperature to central system */
  digitalWrite(GREEN_LIGHT_PIN, HIGH);
  /* Build a TM to send back containing the status of the command execution */
  LbMsg tm(0); tm.setSrc(ID_BOURDILOT_FREEZER_SLAVE); tm.setDst(dst); tm.setCmd(ID_BOURDILOT_FREEZER_NETWORK_TM);
  /* Compute the CRC and send the message */
  tm.compute();
  NRF24_PRINT( Serial.print("Sending Network on NRF24 to dst="); Serial.print(dst); Serial.print("..."); )
  nrf24.stopListening();
  bool writeStatus = nrf24.write(tm.getFrame(), tm.getFrameLen());
  nrf24.startListening();
  NRF24_PRINT(
    if(true == writeStatus) { Serial.println("OK"); } else { Serial.println("ERROR"); }
    Serial.print("Network message sent: "); tm.print(); Serial.println();
  )
  digitalWrite(GREEN_LIGHT_PIN, LOW);
}

void nrf24SendNetworkToLOST(int arg_cnt, char **args) { nrf24SendNetworkTo(ID_LOST_MASTER); }

void setup() {
/* ****************************
 *  Pin configuration
 * ****************************
 */
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
  /* Set Power Amplifier (PA) level to one of four levels: */
  /* RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX */
  /* The power levels correspond to the following output levels respectively: */
  /* NRF24L01: -18dBm, -12dBm,-6dBM, and 0dBm */
  nrf24.setPALevel(RF24_PA_MAX);
  Serial.println("NRF24 PA level set");
  //nrf24.setAddressWidth(4);
  Serial.println("NRF24 address width set");
  /* Open pipe for Master to Slave messages */
  uint8_t nrf24AdrR[6] = "LBM2S";
  nrf24.openReadingPipe(1, nrf24AdrR);
  Serial.println("NRF24 reading pipe opened");
  /* Open pipe for Slave to Master messages */
  uint8_t nrf24AdrW[6] = "LBS2M";
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
  cmdAdd("nrf24SendNetworkLOST", "Send network on NRF24 link", nrf24SendNetworkToLOST);
  cmdAdd("nrf24EnablePrint", "Enable print in NRF24 lib", nrf24EnablePrint);
  cmdAdd("nrf24DisbalePrint", "Disable print in NRF24 lib", nrf24DisablePrint);
  cmdAdd("help", "List commands", cmdList);
*/

  Serial.println("nodeFreeze Init done");

  delay(1000);
  digitalWrite(RED_LIGHT_PIN, LOW);
  digitalWrite(ORANGE_LIGHT_PIN, LOW);
  digitalWrite(GREEN_LIGHT_PIN, LOW);
}

void loop() {
  /* Read incoming messages */
  if(true == nrf24.available()) {
    LbMsg msg(32-4-1); /* 32 bytes max in NRF24 static payload */
    nrf24.read(msg.getFrame(), 32);
    NRF24_PRINT( Serial.print("NRF24 rx: "); msg.print(); )
    if(true == msg.check()) {
      NRF24_PRINT( Serial.println(": OK"); )
      /* Actions to do */
      if(ID_BOURDILOT_FREEZER_SLAVE == msg.getDst()) {
        digitalWrite(ORANGE_LIGHT_PIN, HIGH);
        if(ID_BOURDILOT_FREEZER_NETWORK_TC == msg.getCmd()) {
          nrf24SendNetworkTo(msg.getSrc());
        }
        else if(ID_BOURDILOT_FREEZER_TEMP_TC == msg.getCmd()) {
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
        NRF24_PRINT( Serial.print("NRF24 msg not for me dst="); Serial.println(msg.getDst()); )
      }
    }
    else {
      digitalWrite(RED_LIGHT_PIN, HIGH);
      NRF24_PRINT( Serial.println(": Bad CKS !"); )
      digitalWrite(RED_LIGHT_PIN, LOW);
    }
  }

  /* ****************************
   *  Cyclic tasks
   * ****************************
   */
  if(20000 < nrf24SendTempCycle) { nrf24SendNetworkTo(ID_LOST_MASTER); nrf24SendTempTo(ID_LOST_MASTER); nrf24SendTempCycle = 0; }
  nrf24SendTempCycle++;

  /* Poll for new command line */
  cmdPoll();
  /* Wait a minimum for cyclic task */
  delay(1);
}

