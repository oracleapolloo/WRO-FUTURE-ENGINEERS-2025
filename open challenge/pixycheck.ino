#include <Pixy2.h>

// Initialize Pixy2 object
Pixy2 pixy;

void setup() {
  Serial.begin(9600); // Initialize serial communication at 115200 baud
  Serial.println("Starting...");

  pixy.init(); // Initialize Pixy2 camera
  pixy.changeProg("color_connected_components"); // Set to CCC mode
}

void loop() {
  int i;
  uint16_t blocks = pixy.ccc.getBlocks(); // Get detected blocks

  if (blocks) {
    Serial.println("Detected blocks:");
    for (i = 0; i < blocks; i++) {
      // Check for signature 1 or 2
      if (pixy.ccc.blocks[i].m_signature == 1 || pixy.ccc.blocks[i].m_signature == 2) {
        Serial.print("  Block ");
        Serial.print(i);
        Serial.print(": ");
        
        // Print signature
        Serial.print("Signature=");
        Serial.print(pixy.ccc.blocks[i].m_signature);
        
        // Print x-axis position (center of object, 0 to 316)
        Serial.print(" X=");
        Serial.print(pixy.ccc.blocks[i].m_x);
        
        // Print width (1 to 316)
        Serial.print(" Width=");
        Serial.print(pixy.ccc.blocks[i].m_width);
        
        // Print height (1 to 208)
        Serial.print(" Height=");
        Serial.print(pixy.ccc.blocks[i].m_height);
        
        // Additional data available from Pixy2
        Serial.print(" Y=");
        Serial.print(pixy.ccc.blocks[i].m_y); // y-axis position (0 to 208)
        
        Serial.print(" Angle=");
        Serial.print(pixy.ccc.blocks[i].m_angle); // Angle for color codes (-180 to 180)
        
        Serial.print(" Index=");
        Serial.print(pixy.ccc.blocks[i].m_index); // Tracking index
        
        Serial.print(" Age=");
        Serial.print(pixy.ccc.blocks[i].m_age); // Frames tracked
        
        Serial.println();
        delay(500);
      }
    }
  }
  delay(100); // Small delay to avoid overwhelming serial output
}
