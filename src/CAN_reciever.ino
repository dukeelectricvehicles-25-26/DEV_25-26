#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);

void setup() {
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

    // Motor-controller heartbeat: std ID 0x300, DLC=2, B0=0xAA, B1=counter
    if ( (canMsg.can_id & 0x7FF) == 0x300 && canMsg.can_dlc >= 2 && canMsg.data[0] == 0xAA ) {
      static bool have_last = false;
      static uint8_t last_ctr = 0;
      uint8_t ctr = canMsg.data[1];

      if (!have_last) {
        have_last = true;
        last_ctr = ctr;
        Serial.print("HB id=0x300 ctr=");
        Serial.print(ctr);
        Serial.println(" (first)");
      } else {
        uint8_t delta = (uint8_t)(ctr - last_ctr);   // modulo-256
        if (delta == 0) {
          Serial.print("HB id=0x300 ctr=");
          Serial.print(ctr);
          Serial.println(" DUPLICATE");
        } else if (delta == 1) {
          Serial.print("HB id=0x300 ctr=");
          Serial.print(ctr);
          Serial.println(" OK");
        } else {
          // missed frames = delta - 1 (mod 256). If this is huge, it could be a reset.
          uint8_t missed = (uint8_t)(delta - 1);
          Serial.print("HB id=0x300 ctr=");
          Serial.print(ctr);
          Serial.print(" MISSED=");
          Serial.println(missed);
        }
        last_ctr = ctr;
      }
    } else {
      // Fallback: original raw dump for non-heartbeat frames
      Serial.print(canMsg.can_id, HEX); // ID
      Serial.print(" ");
      Serial.print(canMsg.can_dlc, HEX); // DLC
      Serial.print(" ");
      for (int i = 0; i < canMsg.can_dlc; i++) {
        Serial.print(canMsg.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
}
