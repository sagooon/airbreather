#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
// CRITICAL: Libraries for Anti-Brownout
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

/* =========================================================================
   1. HARDWARE MAPPING & CONFIGURATION
   ========================================================================= */
// --- PINS (MATCHING YOUR WORKING SETUP) ---
#define PIN_ESC     4   
#define PIN_TL      27  // Top-Left
#define PIN_TR      14  // Top-Right
#define PIN_BL      25  // Bottom-Left
#define PIN_BR      26  // Bottom-Right
#define PIN_BUZZER  23  
#define LED_STATUS  2   

// --- SAFETY & LIMITS ---
#define MAX_PHYSICAL_THROTTLE 0.40  // 40% Hard Limit for Indoor Demo
#define ESC_MIN     1000            
#define ESC_MAX     2000            
#define DEMO_MAX_PULSE (ESC_MIN + (int)((ESC_MAX - ESC_MIN) * MAX_PHYSICAL_THROTTLE))

#define SERVO_SOFT_LIMIT 0.25       // 25% mechanical limit
#define WIFI_SSID "ABER_GroundStation"
#define WIFI_PASS "rocketscience"
#define WIFI_CHANNEL 6              // Fixed Channel for Stability

// --- TIMING ---
#define LOOP_FREQ   200             
#define LOOP_TIME_US (1000000 / LOOP_FREQ)

/* =========================================================================
   2. GLOBAL OBJECTS & VARIABLES
   ========================================================================= */
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
WebServer server(80);

Servo esc, tl, tr, bl, br;

// Control Gains (Cascaded PID)
float Kp_outer = 4.5;   // Attitude
float Kp_inner = 0.012; // Rate
float Kd_inner = 0.0006; 

// Kalman Filter
struct Kalman { float Q=0.01, R=0.1, P=1.0, K=0.0, X=0.0; };
Kalman kPitch, kRoll;

// System State
bool isArmed = false;
int throttlePct = 0;       
unsigned long lastLoopTime = 0;
unsigned long lastHeartbeat = 0;

/* =========================================================================
   3. UTILITY FUNCTIONS
   ========================================================================= */
void beep(int duration, int count) {
  for(int i=0; i<count; i++) {
    digitalWrite(PIN_BUZZER, HIGH); digitalWrite(LED_STATUS, HIGH);
    delay(duration);
    digitalWrite(PIN_BUZZER, LOW); digitalWrite(LED_STATUS, LOW);
    delay(duration);
  }
}

float updateKalman(Kalman &k, float measurement) {
  k.P = k.P + k.Q;
  k.K = k.P / (k.P + k.R);
  k.X = k.X + k.K * (measurement - k.X);
  k.P = (1 - k.K) * k.P;
  return k.X;
}

void writeMotor(int pct) {
  int us = map(pct, 0, 100, ESC_MIN, DEMO_MAX_PULSE);
  esc.writeMicroseconds(us);
}

void centerServos() {
  tl.writeMicroseconds(1500); tr.writeMicroseconds(1500);
  bl.writeMicroseconds(1500); br.writeMicroseconds(1500);
}

void performControlSurfaceCheck() {
  Serial.println("Performing Control Surface Check...");
  beep(50, 2); 
  
  int d = 500 * SERVO_SOFT_LIMIT; 
  // Pitch Check
  tl.writeMicroseconds(1500+d); tr.writeMicroseconds(1500+d); bl.writeMicroseconds(1500-d); br.writeMicroseconds(1500-d);
  delay(250);
  tl.writeMicroseconds(1500-d); tr.writeMicroseconds(1500-d); bl.writeMicroseconds(1500+d); br.writeMicroseconds(1500+d);
  delay(250);
  // Roll Check
  tl.writeMicroseconds(1500+d); tr.writeMicroseconds(1500-d); bl.writeMicroseconds(1500+d); br.writeMicroseconds(1500-d);
  delay(250);
  tl.writeMicroseconds(1500-d); tr.writeMicroseconds(1500+d); bl.writeMicroseconds(1500-d); br.writeMicroseconds(1500+d);
  delay(250);
  
  centerServos();
  Serial.println("Check Complete.");
  beep(400, 1); 
}

/* =========================================================================
   4. WEB DASHBOARD
   ========================================================================= */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ABER CONTROL</title>
  <style>
    body { font-family: monospace; text-align: center; background: #111; color: #0f0; padding: 20px;}
    .card { border: 1px solid #333; background: #222; margin: 15px auto; padding: 20px; max-width: 400px; }
    button { padding: 15px; width: 45%; font-size: 18px; font-weight: bold; border: none; cursor: pointer; color: white; margin: 5px; }
    .arm-btn { background: #c0392b; } .disarm-btn { background: #27ae60; }
    input[type=range] { width: 100%; margin: 25px 0; }
  </style>
</head><body>
  <h1>ABER FLIGHT COMPUTER</h1>
  <div class="card">
    <div id="status-box" style="color: grey;">SYSTEM DISARMED</div>
    <button class="arm-btn" onclick="toggleArm(1)">ARM</button>
    <button class="disarm-btn" onclick="toggleArm(0)">DISARM</button>
  </div>
  <div class="card">
    <h3>THROTTLE</h3>
    <input type="range" min="0" max="100" value="0" oninput="updThr(this.value)">
    <div id="thrDisp">0%</div>
  </div>
  <div class="card">PITCH: <span id="p">0.0</span> | ROLL: <span id="r">0.0</span></div>
<script>
  function toggleArm(s) {
    fetch(s ? '/arm' : '/disarm').then(r => {
      document.getElementById("status-box").innerHTML = s ? "<span style='color:red'>!!! ARMED !!!</span>" : "SYSTEM DISARMED";
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
</script></body></html>
)rawliteral";

/* =========================================================================
   5. SETUP ROUTINE
   ========================================================================= */
void setup() {
  // 1. DISABLE BROWNOUT (Crucial for WiFi)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

  Serial.begin(115200);
  Serial.setTimeout(10); // Fast serial reads
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  
  Serial.println("Stabilizing Power (3s)...");
  delay(3000); 
  beep(80, 3); 

  // 2. ACTUATORS (Explicit Timer Allocation)
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);

  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, ESC_MIN, ESC_MAX);
  esc.writeMicroseconds(ESC_MIN);
  
  tl.attach(PIN_TL); tr.attach(PIN_TR);
  bl.attach(PIN_BL); br.attach(PIN_BR);
  centerServos();

  // 3. I2C MANUAL WAKEUP
  Serial.println("Initializing I2C...");
  Wire.begin(21, 22);        
  Wire.setClock(400000);     
  delay(100);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  delay(100); 

  // 4. SENSORS
  if (!mpu.begin()) {
    delay(500); 
    if (!mpu.begin()) { Serial.println("MPU Error - Check Wiring"); }
  }
  bmp.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 5. WIFI
  WiFi.mode(WIFI_AP); 
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // 6. SERVER ROUTES
  server.on("/", [](){ server.send(200, "text/html", index_html); });
  server.on("/arm", [](){ isArmed = true; server.send(200, "text/plain", "ARMED"); });
  server.on("/disarm", [](){ isArmed = false; throttlePct=0; writeMotor(0); centerServos(); server.send(200, "text/plain", "DISARMED"); });
  
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
  
  performControlSurfaceCheck();
  Serial.println("READY. SERIAL BACKUP: 'ARM', 'DISARM', '0-100'");
}

/* =========================================================================
   6. MAIN CONTROL LOOP
   ========================================================================= */
void loop() {
  unsigned long now = micros();

  // --- A. FLIGHT CONTROL LOOP (200Hz) ---
  if (now - lastLoopTime >= LOOP_TIME_US) {
    lastLoopTime = now;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float p_ang = updateKalman(kPitch, atan2(a.acceleration.z, a.acceleration.x) * 57.29);
    float r_ang = updateKalman(kRoll, atan2(a.acceleration.y, a.acceleration.x) * 57.29);
    
    // Control Logic
    if (isArmed && throttlePct > 5) { 
      float p_out = constrain((((0.0 - p_ang) * Kp_outer) - g.gyro.y) * Kp_inner - (g.gyro.y * Kd_inner), -SERVO_SOFT_LIMIT, SERVO_SOFT_LIMIT);
      float r_out = constrain((((0.0 - r_ang) * Kp_outer) - g.gyro.z) * Kp_inner - (g.gyro.z * Kd_inner), -SERVO_SOFT_LIMIT, SERVO_SOFT_LIMIT);

      tl.writeMicroseconds(1500 + (p_out * 500) + (r_out * 500));
      tr.writeMicroseconds(1500 + (p_out * 500) - (r_out * 500));
      bl.writeMicroseconds(1500 - (p_out * 500) + (r_out * 500));
      br.writeMicroseconds(1500 - (p_out * 500) - (r_out * 500));
    } else {
      centerServos();
      if(!isArmed) esc.writeMicroseconds(ESC_MIN); 
    }
  }

  // --- B. WIFI HANDLER ---
  server.handleClient();

  // --- C. SERIAL BACKUP CONTROL ---
  if(Serial.available()) {
    String c = Serial.readStringUntil('\n'); c.trim(); c.toUpperCase();
    if(c == "ARM") { isArmed = true; Serial.println("ARMED (SERIAL)"); }
    else if(c == "DISARM") { isArmed = false; throttlePct=0; writeMotor(0); centerServos(); Serial.println("DISARMED"); }
    else if(isArmed && c.length() > 0 && isDigit(c.charAt(0))) { 
      throttlePct = constrain(c.toInt(), 0, 100); 
      writeMotor(throttlePct); 
      Serial.print("THR: "); Serial.println(throttlePct);
    }
  }

  // --- D. HEARTBEAT ---
  if (isArmed && millis() - lastHeartbeat > 1000) {
    lastHeartbeat = millis();
    digitalWrite(PIN_BUZZER, HIGH); digitalWrite(LED_STATUS, HIGH);
    delay(50);
    digitalWrite(PIN_BUZZER, LOW); digitalWrite(LED_STATUS, LOW);
  }
}
