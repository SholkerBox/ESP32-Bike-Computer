#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h> // Re-added for captive portal
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// --- SET THE WIFI NETWORK YOUR ESP32 WILL CREATE ---
const char* ap_ssid = "ESP32";
const char* ap_password = "12345678"; // Optional password
// --------------------------------------------------

// DNS server
DNSServer dnsServer;

// Web Server on port 80
WebServer server(80);

// MPU6050 Sensor object
Adafruit_MPU6050 mpu;

// --- MEASURE YOUR WHEEL AND UPDATE THIS ---
const int WHEEL_CIRCUMFERENCE_MM = 2105;
// -----------------------------------------

// Hall Effect Sensor Pin
const int hallPin = 15; // Using GPIO15

// Global variables for sensor data
float pitch = 0.0;
float accelY = 0.0;
volatile float speedKph = 0.0;
volatile unsigned long lastMagnetTime = 0;
String magnetState = "Standby";
const int SPEED_TIMEOUT_MS = 2000;

// Interrupt function for Hall sensor
void IRAM_ATTR onMagnetDetect() {
  unsigned long now = millis();
  unsigned long timeElapsed = now - lastMagnetTime;

  if (timeElapsed > 20) { // Debounce
    speedKph = ((float)WHEEL_CIRCUMFERENCE_MM / (float)timeElapsed) * 3.6;
    lastMagnetTime = now;
  }
}

// Function to handle the root webpage (HTML and JavaScript are unchanged)
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Bike Module</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <link href="https://fonts.googleapis.com/css2?family=Roboto+Mono:wght@300;700&family=Roboto:wght@400&display=swap" rel="stylesheet">
  <style>
    html, body {
      height: 100%; margin: 0; padding: 0; background-color: #000; color: #f0f0f0; font-family: 'Roboto Mono', monospace; overflow: hidden;
    }
    .main-container {
      display: flex;
      justify-content: space-between; /* Space out columns */
      align-items: center; /* Vertically center content */
      height: 100%;
      padding: 0 5vw; /* Padding on the sides */
      box-sizing: border-box;
    }
    .column {
      flex: 1; /* Each column takes equal space initially */
      height: 90vh; /* Make columns slightly shorter than screen */
      display: flex;
      flex-direction: column;
      justify-content: center;
      align-items: center;
      padding: 0 1vw;
    }
    .center-column {
      flex: 1.5; /* Make center column slightly wider */
      justify-content: space-around; /* Distribute space within center column */
    }
    .placeholder-box {
      width: 100%;
      height: 50%; /* Adjust height as needed */
      border: 2px dashed #444;
      border-radius: 15px;
      display: flex;
      flex-direction: column;
      justify-content: center;
      align-items: center;
      text-align: center;
      font-family: 'Roboto', sans-serif;
      color: #666;
    }
    .placeholder-box h3 {
      margin-bottom: 10px;
      color: #888;
      font-weight: 400;
    }
    .placeholder-box .icon {
        font-size: 3em; /* Larger icons */
        margin-bottom: 15px;
    }
    .spotify-controls button {
        background: none; border: 1px solid #555; color: #ccc; border-radius: 50%; width: 40px; height: 40px; margin: 5px; font-size: 1.2em; cursor: pointer;
    }

    /* Data Display Styles */
    .magnet-status { font-size: 2.0vh; font-weight: 700; color: #00ff88; height: 4vh; letter-spacing: 2px; margin-bottom: 2vh;}
    .speed-display { font-size: 20vh; color: #FFFFFF; font-weight: 700; line-height: 0.9; } /* White Speed */
    .unit-kmh { font-size: 3vh; color: #888; font-weight: 300; margin-left: 10px; }
    .incline-display { font-size: 7vh; color: #999999; font-weight: 700; margin-top: 1vh; } /* Greyscale Incline */
    .accel-display { font-size: 7vh; color: #BBBBBB; font-weight: 700; margin-top: 1vh; } /* Greyscale Accel */
    .unit-deg, .unit-ms2 { font-size: 2.5vh; color: #666; font-weight: 300; }

    /* Ensure icons load - requires internet on phone */
    @import url('https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css');

  </style>
</head>
<body>
  <div class="main-container">
    
    <!-- Left Column: Spotify Placeholder -->
    <div class="column">
      <div class="placeholder-box">
          <span class="icon"><i class="fab fa-spotify"></i></span>
          <h3>Spotify Control</h3>
          <div>Now Playing: Song Title...</div>
          <div class="spotify-controls">
              <button><i class="fas fa-backward-step"></i></button>
              <button><i class="fas fa-play"></i></button>
              <button><i class="fas fa-forward-step"></i></button>
          </div>
      </div>
    </div>

    <!-- Center Column: Main Data -->
    <div class="column center-column">
      <div id="magnet" class="magnet-status">SYSTEM READY</div>
      <div class="speed-display">
        <span id="speed">0.0</span><span class="unit-kmh">km/h</span>
      </div>
      <div class="incline-display">
        <span id="pitch">0.0</span><span class="unit-deg">&deg;</span>
      </div>
      <div class="accel-display">
        <span id="accelY">0.00</span><span class="unit-ms2">m/s&sup2;</span>
      </div>
    </div>

    <!-- Right Column: GPS Placeholder -->
    <div class="column">
      <div class="placeholder-box">
          <span class="icon"><i class="fas fa-map-marker-alt"></i></span>
          <h3>GPS / Map</h3>
          <div>Lat: --.--</div>
          <div>Lon: --.--</div>
          <div>(Map Area Placeholder)</div>
      </div>
    </div>
  </div>

  <script>
    let audioContext;
    let beepPlayed = false;
    function playBeep() { /* Beep function remains the same */ }
    document.body.addEventListener('touchstart', () => { /* Audio context init remains */ }, { once: true });
    document.body.addEventListener('click', () => { /* Audio context init remains */ }, { once: true });

    setInterval(function() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var data = this.responseText.split(',');
          document.getElementById("speed").innerHTML = data[0];
          document.getElementById("pitch").innerHTML = data[1];
          document.getElementById("accelY").innerHTML = data[3];
          var magnetEl = document.getElementById("magnet");
          var state = data[2];
          if (state == "DETECTED") {
            magnetEl.innerHTML = "!! MAGNET DETECTED !!"; magnetEl.style.color = "#ff4444";
            if (!beepPlayed) { /* playBeep(); Beep logic remains */ beepPlayed = true; }
          } else {
            magnetEl.innerHTML = "SYSTEM READY"; magnetEl.style.color = "#00ff88"; beepPlayed = false;
          }
        }
      };
      xhttp.open("GET", "/data", true); xhttp.send();
    }, 500); 
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// Function to send sensor data (unchanged)
void handleData() {
  String data = String(speedKph, 1) + "," + String(pitch, 1) + "," + magnetState + "," + String(accelY, 2);
  server.send(200, "text/plain", data);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize Hall Sensor
  pinMode(hallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), onMagnetDetect, FALLING);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // --- START ACCESS POINT ---
  Serial.print("Creating Access Point: ");
  Serial.println(ap_ssid);
  WiFi.softAP(ap_ssid, ap_password); // Start the AP

  IPAddress myIP = WiFi.softAPIP(); // Get the IP (will be 192.168.4.1)
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // --- START CAPTIVE PORTAL (DNS SERVER) ---
  dnsServer.start(53, "*", myIP); // Redirect all DNS requests to the ESP32

  // --- Setup Web Server Routes ---
  server.on("/", handleRoot);
  server.on("/data", handleData);
  // Redirect any other request to the main page (Captive Portal trigger)
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1", true); // Redirect to ESP32 IP
    server.send(302, "text/plain", ""); // 302 Found Redirect
  });

  server.begin(); // Start the web server
  Serial.println("HTTP server started.");

  lastMagnetTime = millis(); // Initialize timer
}

void loop() {
  // --- REQUIRED FOR CAPTIVE PORTAL ---
  dnsServer.processNextRequest(); // Handle DNS requests

  // Handle web server requests
  server.handleClient();

  // Read MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate Incline (Pitch)
  pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  
  // Store forward acceleration
  accelY = a.acceleration.y;

  // Check for speed timeout
  if (millis() - lastMagnetTime > SPEED_TIMEOUT_MS) {
    speedKph = 0.0;
  }

  // Check for magnet "flash" state
  if (millis() - lastMagnetTime < 250) {
    magnetState = "DETECTED";
  } else {
    magnetState = "Standby";
  }
}