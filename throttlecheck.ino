#include <ESP32Servo.h>

// --- CONFIGURATION ---
#define PIN_ESC      4     // The pin you are using
#define MIN_PULSE    1000   // 0% Throttle
#define MAX_PULSE    2000   // 100% Throttle
#define MOTOR_KV     2600
#define LIPO_CELLS   3      // 3S Battery
#define VOLTS_PER_CELL 3.7

Servo esc;
bool isArmed = false;
float maxTheoreticalRPM = 0;

void setup() {
  Serial.begin(115200);

  // 1. Calculate Max RPM for display
  maxTheoreticalRPM = LIPO_CELLS * VOLTS_PER_CELL * MOTOR_KV;

  // 2. Setup ESC
  esc.setPeriodHertz(50); 
  esc.attach(PIN_ESC, MIN_PULSE, MAX_PULSE);

  // 3. Initialize at ZERO throttle
  esc.writeMicroseconds(MIN_PULSE);

  Serial.println("\n\n=== DYS 2207 SERIAL THROTTLE ===");
  Serial.println("Type 'arm' to unlock.");
  Serial.println("Type 0 - 100 to set speed.");
  Serial.println("Type 'stop' to cut power.");
  Serial.printf("Max RPM Target: %.0f\n", maxTheoreticalRPM);
  Serial.println("--------------------------------");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 

    // --- SAFETY COMMANDS ---
    if (input.equalsIgnoreCase("arm")) {
      isArmed = true;
      Serial.println(">> SYSTEM ARMED. BE CAREFUL.");
      return;
    }
    
    if (input.equalsIgnoreCase("stop")) {
      isArmed = false;
      esc.writeMicroseconds(MIN_PULSE);
      Serial.println(">> EMERGENCY STOP. Motor Locked.");
      return;
    }

    // --- THROTTLE CONTROL ---
    if (isArmed) {
      // Check if input is a valid number
      if (input.length() > 0 && isDigit(input.charAt(0))) {
        int percent = input.toInt();
        
        // Safety Clamp: Keep it between 0 and 100
        percent = constrain(percent, 0, 100);

        // Map Percentage to Microseconds
        int pwmValue = map(percent, 0, 100, MIN_PULSE, MAX_PULSE);
        esc.writeMicroseconds(pwmValue);

        // Calculate estimated RPM under load (approx 85% efficiency)
        float currentRPM = (maxTheoreticalRPM * (percent / 100.0)) * 0.85;

        Serial.print("Throttle: "); Serial.print(percent); Serial.print("%");
        Serial.print(" | PWM: "); Serial.print(pwmValue); Serial.print("us");
        Serial.print(" | Est. RPM: "); Serial.println(currentRPM, 0);
      }
    } else {
      Serial.println(">> ERROR: System is DISARMED. Type 'arm' first.");
    }
  }
}
