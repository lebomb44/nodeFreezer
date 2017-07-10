#include <Cmd.h>
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(2);
DallasTemperature tempSensors(&oneWire);

void setup() {
  Serial.begin(115200);
  tempSensors.begin();

  cmdInit();

  cmdAdd("homeEasySend", "Send HomeEsay HEX code", homeEasySend);

  Serial.println("nodeFreeze Init OK");
}

void loop() {
  delay(1);
  tempSensors.requestTemperatures();
  Serial.print(tempSensors.)

  if(true == nrf24.available()) {
    uint8_t lbComFrame[LBCOM_FRAME_MAX_SIZE] = {0};
    Serial.print("NRF24 rx:");
    nrf24.read(&lbComFrame[0], 4);
    nrf24.read(&lbComFrame[4], lbComFrame[3]+1);
    if(true == nrf24_printIsEnabled) {
      for(uint16_t i=0; i<(4+lbComFrame[3]+1); i++) { Serial.print(" "); Serial.print(lbComFrame[i], HEX); }
      Serial.println();
    }
    lbCom.send(lbComFrame[0], lbComFrame[1], lbComFrame[2], lbComFrame[3], &lbComFrame[4]);
  }

  cmdPoll();
}

