#include <SPI.h>  //allows communication between Arduino and peripherals using the SPI protocol
#include <mcp2515.h> //this is the library that simplifies controlling the MCP2515 CAN controller chip

struct can_frame canMsg;  //a data structure (from the library) that represents a single CAN message.
//It contains fields like: can_id, can_data length code (number of bytes), data[8] → array for the 8 data bytes in the frame
MCP2515 mcp2515(10);

void setup() {
    Serial.begin(115200); //Starts serial communication with your computer so you can print messages at 115200 baud.
    mcp2515.reset(); //resets the MCP2515 to its default state.
    mcp2515.setBitrate(CAN_125KBPS); //sets the bus speed to 125 kilobits per second (must match your network!).
    mcp2515.setNormalMode(); //puts the MCP2515 into Normal mode, meaning it will actively send/receive CAN frames.

    Serial.println("------- CAN Read ----------");
    Serial.println("ID  DLC  DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    //readMessage(&canMsg) tries to read a frame into the canMsg structure
    //If successful, it returns MCP2515::ERROR_OK
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    for (int i = 0; i<canMsg.can_dlc; i++)  {  
        Serial.print(canMsg.data[i],HEX);
        Serial.print(" ");
    } // Loops through each of the message’s data bytes and prints them in hexadecimal form. Each byte is separated by a space.
        Serial.println();      
  }
}



