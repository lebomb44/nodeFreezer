#include <Cmd.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RF24.h>
#include <ID.h>

#define RED_LIGHT_PIN 5
#define ORANGE_LIGHT_PIN 6
#define GREEN_LIGHT_PIN 7

#define NRF24_CE_pin   9
#define NRF24_CSN_pin  10
#define NRF24_MOSI_pin 11
#define NRF24_MISO_pin 12
#define NRF24_SCK_pin  13

OneWire oneWire(2);
DallasTemperature tempSensors(&oneWire);
RF24 nrf24(NRF24_CE_pin, NRF24_CSN_pin);
bool nrf24_printIsEnabled = false;

/* NRF24 */
void nrf24Send(int arg_cnt, char **args) {
}
void nrf24EnablePrint(int arg_cnt, char **args) { nrf24_printIsEnabled = true; Serial.println("NRF24 print enabled"); }
void nrf24DisablePrint(int arg_cnt, char **args) { nrf24_printIsEnabled = false; Serial.println("NRF24 print disabled"); }


void setup() {
  pinMode(RED_LIGHT_PIN, OUTPUT);
  pinMode(ORANGE_LIGHT_PIN, OUTPUT);
  pinMode(GREEN_LIGHT_PIN, OUTPUT);
  digitalWrite(RED_LIGHT_PIN, HIGH);
  digitalWrite(ORANGE_LIGHT_PIN, HIGH);
  digitalWrite(GREEN_LIGHT_PIN, HIGH);

  Serial.begin(115200);

  tempSensors.begin();

  Serial.print("NRF24 begin...");
  if(true == nrf24.begin()) { Serial.println("OK"); } else { Serial.println("ERROR"); }
  Serial.print("NRF24 begin...");
  //nrf24.setPALevel(RF24_PA_LOW);
  Serial.println("NRF24 PA level set");
  nrf24.setAddressWidth(4);
  Serial.println("NRF24 address width set");
  uint32_t nrf24Adr = ID_BOURDILOT_FRIDGE;
  nrf24.openReadingPipe(1, &nrf24Adr);
  Serial.println("NRF24 reading pipe opened");
  nrf24Adr = ID_LOST;
  nrf24.openWritingPipe(&nrf24Adr);
  Serial.println("NRF24 writing pipe opened");
  nrf24.startListening();
  Serial.println("NRF24 listening started");
  
  cmdInit();

  cmdAdd("nrf24Send", "Send frame on NRF24 link", nrf24Send);
  cmdAdd("nrf24EnablePrint", "Enable print in NRF24 lib", nrf24EnablePrint);
  cmdAdd("nrf24DisbalePrint", "Disable print in NRF24 lib", nrf24DisablePrint);
  cmdAdd("help", "List commands", cmdList);

  Serial.println("nodeFreeze Init OK");

  delay(1000);
  digitalWrite(RED_LIGHT_PIN, LOW);
  digitalWrite(ORANGE_LIGHT_PIN, LOW);
  digitalWrite(GREEN_LIGHT_PIN, LOW);
}

void loop() {
  delay(1000);

  /* Read incoming messages */
  uint8_t lbComFrame[256] = {0};
  if(true == nrf24.available()) {
    digitalWrite(ORANGE_LIGHT_PIN, HIGH);
    Serial.print("NRF24 rx:");
    nrf24.read(&lbComFrame[0], 4);
    nrf24.read(&lbComFrame[4], lbComFrame[3]+1);
    if(true == nrf24_printIsEnabled) {
      for(uint16_t i=0; i<(4+lbComFrame[3]+1); i++) { Serial.print(" "); Serial.print(lbComFrame[i], HEX); }
      Serial.println();
    }
    digitalWrite(ORANGE_LIGHT_PIN, LOW);
  }

  /* Send temperature to central system */
  digitalWrite(GREEN_LIGHT_PIN, HIGH);
  tempSensors.requestTemperatures();
  float tempF = tempSensors.getTempCByIndex(0);
  Serial.println(tempF);
  int16_t tempI = (10.0 * tempF);
  lbComFrame[0] = ID_BOURDILOT_FRIDGE; lbComFrame[1] = ID_LOST; lbComFrame[2] = ID_BOURDILOT_FRIDGE_TEMP_TM; lbComFrame[3] = sizeof(int16_t);
  lbComFrame[4] = 0x00FF & (tempI>>8); lbComFrame[5] = 0x00FF & (tempI>>8); lbComFrame[5] = 0x00;
  nrf24.stopListening();
  nrf24.write(lbComFrame, 4+lbComFrame[3]+1);
  nrf24.startListening();
  digitalWrite(GREEN_LIGHT_PIN, LOW);

  cmdPoll();
}

