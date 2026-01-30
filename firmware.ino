#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>

/* =========================================================================
   1. HARDWARE MAPPING & CONFIGURATION
   ========================================================================= */
// --- PINS ---
#define PIN_ESC     4   // ESC Signal
#define PIN_TL      27  // Top-Left Servo
#define PIN_TR      14  // Top-Right Servo
#define PIN_BL      25  // Bottom-Left Servo
#define PIN_BR      26  // Bottom-Right Servo
#define PIN_BUZZER  23  // Active Buzzer
#define LED_STATUS  2   // Onboard Blue LED

// --- SAFETY & LIMITS ---
#define MAX_PHYSICAL_THROTTLE 0.40  // 40% Hard Limit for Indoor Demo
#define ESC_MIN     1000            // Standard ESC Min (Off)
#define ESC_MAX     2000            // Standard ESC Max (Full)
// Calculate the "Demo Max" pulse width (approx 1400us)
#define DEMO_MAX_PULSE (ESC_MIN + (int)((ESC_MAX - ESC_MIN) * MAX_PHYSICAL_THROTTLE))

#define SERVO_SOFT_LIMIT 0.25       // 25% mechanical limit for TVC vanes
#define WIFI_SSID "ABER_GroundStation"
#define WIFI_PASS "rocketscience"
#define WIFI_CHANNEL 6              // FIXED CHANNEL (Solves visibility issues)

// --- CONTROL LOOP TIMING ---
#define LOOP_FREQ   200               // 200Hz Control Loop
#define LOOP_TIME_US (1000000 / LOOP_FREQ)

/* =========================================================================
   2. GLOBAL OBJECTS & VARIABLES
   ========================================================================= */
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
WebServer server(80);

// Actuators
Servo esc;
Servo tl, tr, bl, br;

// Control Gains (Cascaded PID)
float Kp_outer = 4.5;   // Attitude
float Kp_inner = 0.012; // Rate
float Kd_inner = 0.0006; 

// Kalman Filter Structure
struct Kalman { float Q=0.01, R=0.1, P=1.0, K=0.0, X=0.0; };
Kalman kPitch, kRoll;

// System State
bool isArmed = false;
int throttlePct = 0;       // 0-100% from Web Interface
unsigned long lastLoopTime = 0;
unsigned long lastHeartbeat = 0;

/* =========================================================================
   3. UTILITY FUNCTIONS
   ========================================================================= */
// Non-blocking Tone Helper
void beep(int duration) {
  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(LED_STATUS, HIGH);
  delay(duration);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(LED_STATUS, LOW);
}

void beepSequence(int count, int speed) {
  for(int i=0; i<count; i++) {
    beep(speed);
    delay(speed);
  }
}

// Kalman Update
float updateKalman(Kalman &k, float measurement) {
  k.P = k.P + k.Q;
  k.K = k.P / (k.P + k.R);
  k.X = k.X + k.K * (measurement - k.X);
  k.P = (1 - k.K) * k.P;
  return k.X;
}

// Actuator Wrappers
void writeMotor(int pct) {
  // Map UI 0-100% to Real 0-40% range
  int us = map(pct, 0, 100, ESC_MIN, DEMO_MAX_PULSE);
  esc.writeMicroseconds(us);
}

void centerServos() {
  tl.writeMicroseconds(1500); tr.writeMicroseconds(1500);
  bl.writeMicroseconds(1500); br.writeMicroseconds(1500);
}

// --- SYSTEM CHECK ---
void performControlSurfaceCheck() {
  Serial.println("Performing Control Surface Check...");
  beepSequence(2, 50); 
  
  // 1. Pitch Check
  int deflection = 500 * SERVO_SOFT_LIMIT; // ~125us
  
  // Forward
  tl.writeMicroseconds(1500 + deflection); tr.writeMicroseconds(1500 + deflection);
  bl.writeMicroseconds(1500 - deflection); br.writeMicroseconds(1500 - deflection);
  delay(300);
  
  // Back
  tl.writeMicroseconds(1500 - deflection); tr.writeMicroseconds(1500 - deflection);
  bl.writeMicroseconds(1500 + deflection); br.writeMicroseconds(1500 + deflection);
  delay(300);

  // 2. Roll Check
  // Left
  tl.writeMicroseconds(1500 + deflection); tr.writeMicroseconds(1500 - deflection);
  bl.writeMicroseconds(1500 + deflection); br.writeMicroseconds(1500 - deflection);
  delay(300);

  // Right
  tl.writeMicroseconds(1500 - deflection); tr.writeMicroseconds(1500 + deflection);
  bl.writeMicroseconds(1500 - deflection); br.writeMicroseconds(1500 + deflection);
  delay(300);
  
  centerServos();
  Serial.println("Surface Check Complete.");
  beep(500); 
}

/* =========================================================================
   4. WEB DASHBOARD (HTML)
   ========================================================================= */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ABER Mark I Control</title>
  <style>
    body { font-family: 'Courier New', monospace; text-align: center; background-color: #111; color: #0f0; margin: 0; padding: 20px;}
    h1 { border-bottom: 2px solid #0f0; padding-bottom: 10px; }
    .card { border: 1px solid #333; background: #222; margin: 15px auto; padding: 20px; width: 90%; max-width: 400px; border-radius: 5px; }
    button { padding: 15px; width: 45%; font-size: 18px; font-weight: bold; border: none; border-radius: 4px; cursor: pointer; color: white; margin: 5px; }
    .arm-btn { background: #c0392b; }
    .disarm-btn { background: #27ae60; }
    input[type=range] { width: 100%; margin: 25px 0; }
    .telem-val { font-size: 1.5rem; color: #3498db; }
    #status-box { font-size: 1.2rem; font-weight: bold; margin-bottom: 15px; }
  </style>
</head>
<body>
  <h1>ABER FLIGHT COMPUTER</h1>
  
  <div class="card">
    <div id="status-box" style="color: grey;">SYSTEM DISARMED</div>
    <button class="arm-btn" onclick="toggleArm(1)">ARM</button>
    <button class="disarm-btn" onclick="toggleArm(0)">DISARM</button>
  </div>

  <div class="card">
    <h3>THROTTLE GOVERNOR</h3>
    <input type="range" min="0" max="100" value="0" id="thrSlide" oninput="updThr(this.value)">
    <div id="thrDisp" class="telem-val">0%</div>
  </div>

  <div class="card">
    <h3>GYRO TELEMETRY</h3>
    <p>PITCH: <span id="p" class="telem-val">0.0</span> | ROLL: <span id="r" class="telem-val">0.0</span></p>
  </div>

<script>
  function toggleArm(s) {
    fetch(s ? '/arm' : '/disarm').then(r => {
      document.getElementById("status-box").innerHTML = s ? "<span style='color:red'>!!! ARMED !!!</span>" : "<span style='color:grey'>SYSTEM DISARMED</span>";
    });
  }
  function updThr(v) {
    document.getElementById("thrDisp").innerHTML = v + "%";
    fetch('/set_throttle?val=' + v);
  }
  setInterval(() => {
    fetch('/telemetry').then(r => r.json()).then(d => {
      document.getElementById("p").innerText = d.p;
      document.getElementById("r").innerText = d.r;
    });
  }, 250);
</script>
</body>
</html>
)rawliteral";

/* =========================================================================
   5. SETUP ROUTINE
   ========================================================================= */
void setup() {
  Serial.begin(115200);
  
  // 1. HARDWARE INIT & SAFETY DELAY
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  
  // [CRITICAL] 3-SECOND DELAY TO PREVENT BROWNOUT ON WIFI START
  Serial.println("Stabilizing Power (3s)...");
  delay(3000); 

  Serial.println("Booting ABER Mark I...");
  beepSequence(3, 80); 

  // 2. TIMER & ACTUATOR ALLOCATION
  // Explicitly allocate timers to prevent Servo/EDF conflicts
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, ESC_MIN, ESC_MAX);
  esc.writeMicroseconds(ESC_MIN);
  
  tl.attach(PIN_TL); tr.attach(PIN_TR);
  bl.attach(PIN_BL); br.attach(PIN_BR);
  centerServos();

  /* --- I2C FIX: MANUAL WAKEUP --- */
  Serial.println("Initializing I2C Bus...");
  Wire.begin(21, 22);        
  Wire.setClock(400000);     
  delay(100);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  delay(100); 
  /* ------------------------------ */

  // 3. SENSOR INIT
  if (!mpu.begin()) {
    delay(500); // Retry
    if (!mpu.begin()) {
      Serial.println("MPU6050 FATAL ERROR! Check Wiring.");
      while(1) { beep(200); delay(200); }
    }
  }
  
  if (!bmp.begin()) {
     Serial.println("BMP180 Not Found");
  } else {
     Serial.println("BMP180 Online");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("Sensors Ready.");
  
  // 4. CONTROL SURFACE CHECK
  performControlSurfaceCheck();

  // 5. WIFI STARTUP (FIXED)
  Serial.print("Starting WiFi AP...");
  WiFi.mode(WIFI_AP); 
  // FORCE CHANNEL 6 for better visibility
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  Serial.print(" IP: "); Serial.println(WiFi.softAPIP());

  // 6. SERVER ROUTES
  server.on("/", [](){ server.send(200, "text/html", index_html); });
  
  server.on("/arm", [](){ 
    isArmed = true; 
    server.send(200, "text/plain", "ARMED"); 
  });
  
  server.on("/disarm", [](){ 
    isArmed = false; 
    writeMotor(0); 
    centerServos(); 
    server.send(200, "text/plain", "DISARMED"); 
  });
  
  server.on("/set_throttle", [](){
    if (isArmed && server.hasArg("val")) {
      throttlePct = server.arg("val").toInt();
      writeMotor(throttlePct);
    }
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/telemetry", [](){
    String json = "{\"p\":\"" + String(kPitch.X, 1) + "\", \"r\":\"" + String(kRoll.X, 1) + "\"}";
    server.send(200, "application/json", json);
  });

  server.begin();
  
  // Final Ready Signal
  beepSequence(3, 50);
  Serial.println("SYSTEM READY.");
}

/* =========================================================================
   6. MAIN CONTROL LOOP
   ========================================================================= */
void loop() {
  unsigned long now = micros();

  // --- A. FLIGHT CONTROL LOOP (200Hz STRICT) ---
  if (now - lastLoopTime >= LOOP_TIME_US) {
    lastLoopTime = now;

    // 1. Read Sensors
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 2. Kalman Filter (Attitude Estimation)
    float pitchAng = updateKalman(kPitch, atan2(a.acceleration.z, a.acceleration.x) * 57.29);
    float rollAng  = updateKalman(kRoll, atan2(a.acceleration.y, a.acceleration.x) * 57.29);
    
    // 3. Control Logic
    if (isArmed && throttlePct > 5) { 
      // Attitude P-Loop
      float p_target = (0.0 - pitchAng) * Kp_outer;
      float r_target = (0.0 - rollAng)  * Kp_outer;

      // Rate PD-Loop
      float p_out = ((p_target - g.gyro.y) * Kp_inner) - (g.gyro.y * Kd_inner); // Axis Swap for MPU orientation
      float r_out = ((r_target - g.gyro.z) * Kp_inner) - (g.gyro.z * Kd_inner);

      // Constraints
      p_out = constrain(p_out, -SERVO_SOFT_LIMIT, SERVO_SOFT_LIMIT);
      r_out = constrain(r_out, -SERVO_SOFT_LIMIT, SERVO_SOFT_LIMIT);

      // Mixer (X-Tail)
      int tl_s = 1500 + (p_out * 500) + (r_out * 500);
      int tr_s = 1500 + (p_out * 500) - (r_out * 500);
      int bl_s = 1500 - (p_out * 500) + (r_out * 500);
      int br_s = 1500 - (p_out * 500) - (r_out * 500);

      tl.writeMicroseconds(tl_s); tr.writeMicroseconds(tr_s);
      bl.writeMicroseconds(bl_s); br.writeMicroseconds(br_s);
      
    } else if (!isArmed) {
      centerServos();
      esc.writeMicroseconds(ESC_MIN); 
    }
  }

  // --- B. BACKGROUND TASKS ---
  server.handleClient();

  // Armed Heartbeat (Beep every 1 sec)
  if (isArmed) {
    if (millis() - lastHeartbeat > 1000) {
      lastHeartbeat = millis();
      digitalWrite(PIN_BUZZER, HIGH); 
      digitalWrite(LED_STATUS, HIGH);
    } else if (millis() - lastHeartbeat > 50) {
      digitalWrite(PIN_BUZZER, LOW); 
      digitalWrite(LED_STATUS, LOW);
    }
  } else {
    digitalWrite(LED_STATUS, LOW);
  }
}
