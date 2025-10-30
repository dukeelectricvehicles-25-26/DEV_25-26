/* CAN heartbeat receiver for MCP2515 (ID 0x300, DLC=2, B0=0xAA, B1=counter)
 *
 * What this sketch does:
 *   - Initializes the MCP2515 over SPI at 125 kbps (assuming a 16 MHz crystal).
 *   - Programs acceptance masks/filters so ONLY ID 0x300 frames are received.
 *   - Verifies payload format (DLC=2 and first byte == 0xAA).
 *   - Tracks the heartbeat counter to detect duplicates or missed frames.
 *   - Prints both the raw frame and a parsed summary line.
 */

#include <SPI.h>       // SPI peripheral interface (MCP2515 talks over SPI)
#include <mcp2515.h>   // Small library that had MCP2515 commands
// Instantiate an MCP2515 object with chip select on pin 10 (for arduino)
MCP2515 mcp2515(10);

// Struct that holds a single received CAN frame.
// Fields you’ll use here:
//   - can_id
//   - can_dlc: Data Length Code (0..8)
//   - data[8]: up to 8 payload bytes
struct can_frame rx;

// Stores the most recent heartbeat counter we saw, so we can detect gaps.
// Initialized to -1 (meaning “haven’t seen any frame yet”).
int last_counter = -1;

void setup() {
  // Bring up serial for debug prints. 115200 is fast and common.
  Serial.begin(115200);
  // Some boards need the next line to wait for the port.
  while (!Serial) {}

  // Reset MCP2515 to a clean, known state (clears modes, buffers, etc.).
  mcp2515.reset();

  // Configure the CAN bitrate.
  //   - First argument: target bus speed (here 125 kbps).
  //   - Second argument: crystal frequency on your MCP2515 module.
  // Most blue breakout boards are 16 MHz
  if (mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
    // If this prints, your speed/crystal doesn’t match hardware or wiring is wrong.
    Serial.println("Bitrate set failed (check crystal, wiring, or target bitrate).");
  }

  // ---- Acceptance filtering ----
  // Goal: accept ONLY standard (11-bit) frames with ID 0x300.
  //
  // MCP2515 has two masks (MASK0, MASK1) and six filters (RXF0..RXF5).
  // Mask bit = 1  => compare this bit against the filter’s corresponding bit.
  // Mask bit = 0  => “don’t care” (ignore that bit when matching).
  //
  // We set the mask so all 11 ID bits matter (0x7FF = 11 ones).
  // The “false” argument means “standard ID” (not extended).
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);

  // Set every filter to match ID 0x300 exactly.
  // This way, both receive buffers will only accept frames with ID 0x300.
  mcp2515.setFilter(MCP2515::RXF0, false, 0x300);
  mcp2515.setFilter(MCP2515::RXF1, false, 0x300);
  mcp2515.setFilter(MCP2515::RXF2, false, 0x300);
  mcp2515.setFilter(MCP2515::RXF3, false, 0x300);
  mcp2515.setFilter(MCP2515::RXF4, false, 0x300);
  mcp2515.setFilter(MCP2515::RXF5, false, 0x300);

  // Put MCP2515 into normal mode so it actually participates on the bus.
  // (Other modes: loopback, listen-only, configuration.)
  mcp2515.setNormalMode();

  // Header prints for human-readable output.
  Serial.println("---- CAN RX (expect ID 0x300 heartbeat) ----");
  Serial.println("ID     DLC  DATA                 | Parsed");
}

void loop() {
  // Try to pull a frame from the MCP2515 receive buffers into 'rx'.
  // Returns MCP2515::ERROR_OK if a frame was available and successfully read.
  if (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {

    // Sanity 1: ignore extended (29-bit) and RTR (remote) frames.
    // Library sets bits in can_id to indicate these conditions.
    if ((rx.can_id & CAN_EFF_FLAG) || (rx.can_id & CAN_RTR_FLAG)) {
      return; // Not what we expect for the heartbeat; drop it.
    }

    // Extract the 11-bit standard ID (strip off flag bits).
    uint16_t id = rx.can_id & CAN_SFF_MASK;

    // Sanity 2: extra guard even though filters already enforce this.
    if (id != 0x300) {
      return; // Wrong ID; ignore.
    }

    // ---------- RAW PRINT SECTION ----------
    // Print ID in hex, left-padded so columns line up a bit.
    Serial.print("0x");
    if (id < 0x100) Serial.print('0'); // simple pad for 0x0XX range
    Serial.print(id, HEX);
    Serial.print("   ");

    // Print DLC (0..8) in decimal.
    Serial.print(rx.can_dlc, DEC);
    Serial.print("    ");

    // Print each data byte in two-digit hex with spaces.
    for (uint8_t i = 0; i < rx.can_dlc; i++) {
      if (rx.data[i] < 0x10) Serial.print('0'); // pad single hex digit
      Serial.print(rx.data[i], HEX);
      Serial.print(' ');
    }

    // ---------- PARSE + VALIDATE PAYLOAD ----------
    // For the heartbeat we expect:
    //   - DLC == 2 bytes total
    //   - Byte 0 (B0) == 0xAA (marker/tag)
    //   - Byte 1 (B1) == 0..255 counter that increments and wraps
    bool dlc_ok = (rx.can_dlc == 2);
    bool tag_ok = dlc_ok && (rx.data[0] == 0xAA);

    // Build a short, human-friendly summary.
    String parsed = "";

    if (tag_ok) {
      // Pull the counter (B1).
      uint8_t ctr = rx.data[1];

      // Always report the tag and current counter value.
      parsed += "tag=0xAA ctr=";
      parsed += String(ctr);

      // If we’ve seen at least one counter before, compare continuity.
      if (last_counter >= 0) {
        // Subtraction in uint8_t naturally handles wrap (e.g., 0 -> 255 gives diff=255).
        uint8_t diff = (uint8_t)(ctr - (uint8_t)last_counter);

        if (diff == 0) {
          // Same counter twice in a row → duplicate frame or the sender reset.
          parsed += "  [DUPLICATE/RESET?]";
        } else if (diff != 1) {
          // We skipped (diff-1) counters before reaching this one.
          // Example: last=10, now=13 → diff=3 → missed=2 frames.
          uint8_t missed = diff - 1;
          parsed += "  [MISSED ";
          parsed += String(missed);
          parsed += "]";
        }
      }

      // Update our memory of the last-seen counter for next comparison.
      last_counter = ctr;

    } else {
      // Payload didn’t match our heartbeat contract.
      // This helps you catch wrong sender format or wiring/speed errors.
      parsed += "INVALID HB (expect DLC=2, B0=0xAA)";
    }

    // Finish the line: raw frame on the left, parsed interpretation on the right.
    Serial.print(" | ");
    Serial.println(parsed);
  }

  // If no frame was available, loop() returns immediately and tries again.
  // You can add a small delay here (e.g., delay(1)) if you want to throttle CPU usage.
}