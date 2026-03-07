#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// --- FIREBASE CONFIG ---
#define FIREBASE_HOST "esp-cycada-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyAQ4LN3c2Ykydm47I8Hjf-X2Mrl26KBYr0"

// --- WIFI SETTINGS ---
const char* wifi_ssid = "WIFI_NAME";
const char* wifi_password = "WIFI_PASS";

// --- CYCLIST INFO ---
const float CYCLIST_WEIGHT_KG = 70.0;
const int WHEEL_CIRCUMFERENCE_MM = 2105;

// Web Server
WebServer server(80);

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// MPU6050
Adafruit_MPU6050 mpu;

// Hall Effect Sensor
const int hallPin = 15;

// Speed variables
volatile unsigned long lastMagnetTime = 0;
volatile unsigned long rotationCount = 0;
volatile float hallSpeedKph = 0.0;

// IMU velocity integration
float imuVelocityMS = 0.0;
float imuSpeedKph = 0.0;

// Fused speed and animated display
float fusedSpeedKph = 0.0;
float displaySpeedKph = 0.0;
unsigned long lastAnimationTime = 0;
const unsigned long ANIMATION_STEP_MS = 200;
const float ANIMATION_STEP_VALUE = 0.5;

// Timing
const int SPEED_TIMEOUT_MS = 2000;
unsigned long lastIMUUpdate = 0;
unsigned long lastSampleTime = 0;

// IMU variables
float pitch = 0.0;
float roll = 0.0;
float zIncline = 0.0;
float accelY = 0.0;

// Trip tracking
unsigned long tripStartTime = 0;
unsigned long tripDuration = 0;
float tripDistance = 0.0;
float caloriesBurned = 0.0;
bool tripActive = false;
String magnetState = "Standby";

// Buffering
const int BUFFER_SIZE = 30;
float speedArray[BUFFER_SIZE];
float inclineArray[BUFFER_SIZE];
float caloriesArray[BUFFER_SIZE];
int bufferIndex = 0;

// Pending batches queue
struct Batch {
  float speed[BUFFER_SIZE];
  float incline[BUFFER_SIZE];
  float calories[BUFFER_SIZE];
};
const int MAX_PENDING = 4;
Batch pendingBatches[MAX_PENDING];
int pendingHead = 0;
int pendingTail = 0;
int pendingCount = 0;

// Firebase update timing
unsigned long lastFirebaseUpdate = 0;
const unsigned long FIREBASE_UPDATE_INTERVAL = 10000;

// ─────────────────────────────────────────────────────────────
// NEW CYCADA UI — stored in flash to avoid RAM pressure
// ─────────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0">
<title>Cycada</title>
<link href="https://fonts.googleapis.com/css2?family=Syne:wght@400;600;700;800&family=JetBrains+Mono:wght@300;400;500;700&display=swap" rel="stylesheet">
<style>
  :root {
    --orange:#F05A28; --orange-light:#FF7A4D;
    --orange-glow:rgba(240,90,40,0.35);
    --cream:#F5E6D3; --cream-dim:rgba(245,230,211,0.55);
    --cream-faint:rgba(245,230,211,0.10);
    --bg:#0A0A0B; --surface:#111113; --surface2:#18181C;
    --border:rgba(245,230,211,0.08); --white:#FFFFFF;
  }
  *{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent;}
  html,body{width:100%;height:100%;overflow:hidden;}
  body{background:var(--bg);color:var(--cream);font-family:'Syne',sans-serif;display:flex;flex-direction:column;}
  body::before{content:'';position:fixed;top:-80px;left:-80px;width:320px;height:320px;
    background:radial-gradient(circle,rgba(240,90,40,0.12) 0%,transparent 70%);
    pointer-events:none;z-index:0;}

  /* TOP BAR */
  .topbar{display:flex;align-items:center;justify-content:space-between;
    padding:8px 16px;border-bottom:1px solid var(--border);
    position:relative;z-index:10;flex-shrink:0;}
  .logo{display:flex;align-items:center;gap:7px;}
  .logo-icon{width:26px;height:26px;
    background:linear-gradient(135deg,var(--orange-light),var(--orange));
    border-radius:7px;display:flex;align-items:center;justify-content:center;font-size:12px;}
  .logo-text{font-size:16px;font-weight:700;color:var(--cream);letter-spacing:-0.3px;}
  .topbar-right{display:flex;align-items:center;gap:9px;}
  .status-pill{display:flex;align-items:center;gap:5px;padding:4px 10px;
    border-radius:20px;background:var(--cream-faint);border:1px solid var(--border);
    font-family:'JetBrains Mono',monospace;font-size:9px;font-weight:500;
    letter-spacing:0.5px;color:var(--cream-dim);transition:all 0.4s;}
  .status-pill.active{background:rgba(240,90,40,0.15);border-color:rgba(240,90,40,0.4);color:var(--orange-light);}
  .status-dot{width:5px;height:5px;border-radius:50%;background:currentColor;}
  .status-pill.active .status-dot{animation:pulse-dot 1.2s ease-in-out infinite;}
  @keyframes pulse-dot{0%,100%{opacity:1;transform:scale(1);}50%{opacity:0.3;transform:scale(0.6);}}
  .sync-badge{font-family:'JetBrains Mono',monospace;font-size:9px;padding:3px 9px;border-radius:20px;border:1px solid;}
  .sync-badge.offline{color:rgba(245,230,211,0.3);border-color:rgba(245,230,211,0.1);}
  .sync-badge.online{color:#4ade80;border-color:rgba(74,222,128,0.3);background:rgba(74,222,128,0.07);}

  /* MAIN 3-COL */
  .main{flex:1;display:grid;grid-template-columns:200px 1fr 190px;overflow:hidden;position:relative;z-index:10;min-height:0;}
  .col-left{border-right:1px solid var(--border);padding:10px 12px;display:flex;flex-direction:column;gap:7px;}
  .col-mid{padding:8px 14px;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:6px;}
  .col-right{border-left:1px solid var(--border);padding:10px 12px;display:flex;flex-direction:column;gap:7px;}

  /* SPEED RING */
  .speed-ring-wrap{position:relative;width:180px;height:180px;flex-shrink:0;}
  .speed-ring-svg{position:absolute;top:0;left:0;width:100%;height:100%;transform:rotate(-225deg);}
  .ring-track{fill:none;stroke:var(--cream-faint);stroke-width:7;stroke-linecap:round;}
  .ring-progress{fill:none;stroke:url(#ringGrad);stroke-width:7;stroke-linecap:round;
    stroke-dasharray:0 628;transition:stroke-dasharray 0.45s cubic-bezier(0.4,0,0.2,1);}
  .speed-center{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;}
  .speed-label{font-family:'JetBrains Mono',monospace;font-size:9px;letter-spacing:2px;
    color:var(--cream-dim);text-transform:uppercase;margin-bottom:1px;}
  .speed-value{font-family:'JetBrains Mono',monospace;font-size:52px;font-weight:700;
    color:var(--white);line-height:1;letter-spacing:-2px;transition:color 0.3s,text-shadow 0.3s;}
  .speed-value.active{color:var(--orange-light);
    text-shadow:0 0 28px var(--orange-glow),0 0 56px rgba(240,90,40,0.18);}
  .speed-unit{font-family:'JetBrains Mono',monospace;font-size:10px;color:var(--cream-dim);letter-spacing:1px;}
  .speed-meta{display:flex;gap:14px;font-family:'JetBrains Mono',monospace;font-size:9.5px;color:var(--cream-dim);}
  .speed-meta span{color:var(--orange);font-weight:700;}

  /* SPARKLINE */
  .sparkline-card{background:var(--surface);border:1px solid var(--border);border-radius:11px;
    padding:9px 11px;width:100%;flex:1;min-height:0;display:flex;flex-direction:column;}
  .sparkline-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:5px;}
  .spark-label{font-family:'JetBrains Mono',monospace;font-size:8px;letter-spacing:1.5px;
    color:var(--cream-dim);text-transform:uppercase;}
  .sparkline-svg{width:100%;flex:1;min-height:0;}

  /* STAT CARDS */
  .stat-card{background:var(--surface);border:1px solid var(--border);border-radius:11px;
    padding:10px 12px;position:relative;overflow:hidden;flex-shrink:0;}
  .stat-card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;
    background:linear-gradient(90deg,var(--orange),transparent);opacity:0;transition:opacity 0.3s;}
  .stat-card.highlight::before{opacity:1;}
  .stat-card.grow{flex:1;}
  .stat-row{display:flex;align-items:center;justify-content:space-between;}
  .stat-name{font-family:'JetBrains Mono',monospace;font-size:8px;letter-spacing:1.5px;
    color:var(--cream-dim);text-transform:uppercase;margin-bottom:3px;}
  .stat-number{font-family:'JetBrains Mono',monospace;font-size:22px;font-weight:700;
    color:var(--white);line-height:1;letter-spacing:-0.5px;}
  .stat-sub{font-family:'JetBrains Mono',monospace;font-size:8.5px;color:var(--cream-dim);margin-top:1px;}
  .stat-icon{font-size:14px;}
  .stat-card.duration{flex:1;display:flex;flex-direction:column;justify-content:center;}
  .stat-card.duration .stat-number{font-size:26px;letter-spacing:-1px;}

  /* INCLINE */
  .incline-block{display:flex;align-items:center;gap:9px;margin-top:4px;}
  .incline-track-v{width:5px;height:54px;background:var(--cream-faint);
    border-radius:3px;overflow:hidden;display:flex;align-items:flex-end;flex-shrink:0;}
  .incline-fill-v{width:100%;height:0%;
    background:linear-gradient(0deg,var(--orange),var(--orange-light));
    border-radius:3px;transition:height 0.6s cubic-bezier(0.4,0,0.2,1);
    box-shadow:0 0 8px var(--orange-glow);}
  .incline-number{font-family:'JetBrains Mono',monospace;font-size:20px;font-weight:700;color:var(--cream);line-height:1.1;}

  /* BUTTONS */
  .btn-group{display:flex;gap:7px;flex-shrink:0;}
  .btn{flex:1;border:none;border-radius:11px;padding:12px 8px;
    font-family:'Syne',sans-serif;font-size:12px;font-weight:800;
    letter-spacing:0.5px;cursor:pointer;transition:all 0.2s cubic-bezier(0.4,0,0.2,1);}
  .btn:active{transform:scale(0.95);}
  .btn-start{background:linear-gradient(135deg,var(--orange-light),var(--orange));color:white;box-shadow:0 5px 18px var(--orange-glow);}
  .btn-start.running{background:var(--surface2);box-shadow:none;border:1px solid var(--border);color:var(--cream-dim);}
  .btn-stop{background:var(--surface2);color:var(--cream-dim);border:1px solid var(--border);}
  .btn-stop.active-stop{background:rgba(239,68,68,0.1);border-color:rgba(239,68,68,0.4);color:#f87171;}

  @keyframes fadeUp{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:translateY(0)}}
  .topbar{animation:fadeUp 0.3s ease both;}
  .col-left{animation:fadeUp 0.3s 0.05s ease both;}
  .col-mid{animation:fadeUp 0.3s 0.10s ease both;}
  .col-right{animation:fadeUp 0.3s 0.15s ease both;}
</style>
</head>
<body>

<div class="topbar">
  <div class="logo">
    <div class="logo-icon">&#x1F6B4;</div>
    <span class="logo-text">Cycada</span>
  </div>
  <div class="topbar-right">
    <div class="sync-badge offline" id="syncBadge">OFFLINE</div>
    <div class="status-pill" id="statusPill">
      <div class="status-dot"></div>
      <span id="statusText">IDLE</span>
    </div>
  </div>
</div>

<div class="main">

  <!-- LEFT -->
  <div class="col-left">
    <div class="stat-card duration" id="durationCard">
      <div class="stat-name">Duration</div>
      <div class="stat-number" id="durationVal">00:00:00</div>
      <div class="stat-sub" id="tripStatus">Not started</div>
    </div>
    <div class="stat-card">
      <div class="stat-row">
        <div><div class="stat-name">Distance</div><div class="stat-number" id="distVal">0.00</div><div class="stat-sub">km</div></div>
        <span class="stat-icon">&#x1F4CD;</span>
      </div>
    </div>
    <div class="stat-card highlight">
      <div class="stat-row">
        <div><div class="stat-name">Calories</div><div class="stat-number" id="calVal">0</div><div class="stat-sub">kcal</div></div>
        <span class="stat-icon">&#x1F525;</span>
      </div>
    </div>
    <div class="btn-group">
      <button class="btn btn-start" id="btnStart" onclick="startTrip()">START</button>
      <button class="btn btn-stop"  id="btnStop"  onclick="stopTrip()">STOP</button>
    </div>
  </div>

  <!-- MID -->
  <div class="col-mid">
    <div class="speed-ring-wrap">
      <svg class="speed-ring-svg" viewBox="0 0 160 160">
        <defs>
          <linearGradient id="ringGrad" x1="0%" y1="0%" x2="100%" y2="0%">
            <stop offset="0%" style="stop-color:#FF7A4D"/>
            <stop offset="100%" style="stop-color:#F05A28"/>
          </linearGradient>
        </defs>
        <circle class="ring-track" cx="80" cy="80" r="74" stroke-dasharray="471 157"/>
        <circle class="ring-progress" id="ringProgress" cx="80" cy="80" r="74"/>
      </svg>
      <div class="speed-center">
        <div class="speed-label">SPEED</div>
        <div class="speed-value" id="speedVal">0.0</div>
        <div class="speed-unit">km/h</div>
      </div>
    </div>
    <div class="speed-meta">
      MAX <span id="maxSpeed">0.0</span> km/h &nbsp;&middot;&nbsp; AVG <span id="avgSpeedTop">0.0</span> km/h
    </div>
    <div class="sparkline-card">
      <div class="sparkline-header">
        <span class="spark-label">Speed History</span>
        <span class="spark-label" id="sparkMax" style="color:var(--orange)">-- peak</span>
      </div>
      <svg class="sparkline-svg" viewBox="0 0 400 36" preserveAspectRatio="none">
        <defs>
          <linearGradient id="sparkFill" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stop-color="#F05A28" stop-opacity="0.35"/>
            <stop offset="100%" stop-color="#F05A28" stop-opacity="0"/>
          </linearGradient>
        </defs>
        <path id="sparkFillPath" fill="url(#sparkFill)" d="M0,36 L400,36 Z"/>
        <path id="sparkPath" fill="none" stroke="#F05A28" stroke-width="1.8" stroke-linejoin="round" d=""/>
      </svg>
    </div>
  </div>

  <!-- RIGHT -->
  <div class="col-right">
    <div class="stat-card">
      <div class="stat-name">Incline</div>
      <div class="incline-block">
        <div class="incline-track-v"><div class="incline-fill-v" id="inclineFill"></div></div>
        <div><div class="incline-number" id="inclineVal">0.0&deg;</div><div class="stat-sub">grade</div></div>
      </div>
    </div>
    <div class="stat-card">
      <div class="stat-row">
        <div><div class="stat-name">Elevation</div><div class="stat-number" id="elevVal">0</div><div class="stat-sub">m gained</div></div>
        <span class="stat-icon">&#x1F3D4;</span>
      </div>
    </div>
    <div class="stat-card">
      <div class="stat-row">
        <div><div class="stat-name">Magnet</div><div class="stat-number" id="magnetVal">--</div><div class="stat-sub">state</div></div>
        <span class="stat-icon">&#x26A1;</span>
      </div>
    </div>
    <div class="stat-card grow" style="display:flex;flex-direction:column;justify-content:center;">
      <div class="stat-name" style="margin-bottom:5px;">Firebase</div>
      <div style="font-family:'JetBrains Mono',monospace;font-size:10px;color:var(--cream-dim);" id="syncSub">Last update: never</div>
    </div>
  </div>

</div>

<script>
  var tripActive=false, tripStart=null, tripDuration=0;
  var maxSpeed=0, speedSum=0, speedSamples=0, elevGained=0;
  var sparkHistory=[], MAX_SPARK=80;

  function fmt(ms){
    var t=Math.floor(ms/1000);
    var h=String(Math.floor(t/3600)).padStart(2,'0');
    var m=String(Math.floor((t%3600)/60)).padStart(2,'0');
    var s=String(t%60).padStart(2,'0');
    return h+':'+m+':'+s;
  }

  function startTrip(){
    fetch('/trip?action=start').then(function(r){return r.text();}).then(function(){
      tripActive=true; tripStart=Date.now();
      maxSpeed=0; speedSum=0; speedSamples=0; elevGained=0; sparkHistory=[];
      document.getElementById('statusPill').classList.add('active');
      document.getElementById('statusText').textContent='RIDING';
      document.getElementById('btnStart').classList.add('running');
      document.getElementById('btnStart').textContent='RIDING';
      document.getElementById('btnStop').classList.add('active-stop');
      document.getElementById('durationCard').classList.add('highlight');
    });
  }

  function stopTrip(){
    fetch('/trip?action=stop').then(function(r){return r.text();}).then(function(){
      tripActive=false;
      document.getElementById('statusPill').classList.remove('active');
      document.getElementById('statusText').textContent='STOPPED';
      document.getElementById('speedVal').textContent='0.0';
      document.getElementById('speedVal').classList.remove('active');
      document.getElementById('ringProgress').style.strokeDasharray='0 628';
      document.getElementById('btnStart').classList.remove('running');
      document.getElementById('btnStart').textContent='START';
      document.getElementById('btnStop').classList.remove('active-stop');
      document.getElementById('durationCard').classList.remove('highlight');
      document.getElementById('tripStatus').textContent='Not started';
    });
  }

  function drawSparkline(){
    if(sparkHistory.length<2) return;
    var W=400,H=36,pad=2;
    var maxV=Math.max.apply(null,sparkHistory);
    if(maxV<1) maxV=1;
    document.getElementById('sparkMax').textContent=maxV.toFixed(1)+' peak';
    var pts=sparkHistory.map(function(v,i){
      var x=(i/(MAX_SPARK-1))*W;
      var y=H-pad-((v/maxV)*(H-pad*2));
      return x+','+y;
    });
    document.getElementById('sparkPath').setAttribute('d','M'+pts.join(' L'));
    document.getElementById('sparkFillPath').setAttribute('d','M0,'+H+' L'+pts.join(' L')+' L'+W+','+H+' Z');
  }

  setInterval(function(){
    fetch('/data').then(function(r){return r.text();}).then(function(data){
      var p=data.split(',');
      var spd=parseFloat(p[0])||0;
      var inc=parseFloat(p[1])||0;
      var dur=parseInt(p[2])||0;
      var dist=parseFloat(p[3])||0;
      var avg=parseFloat(p[4])||0;
      var cal=parseFloat(p[5])||0;
      var fb=p[6]?p[6].trim():'Offline';
      var lastUpd=p[7]?p[7].trim():'Never';
      var magnet=p[8]?p[8].trim():'--';
      var active=p[9]?p[9].trim():'false';

      // speed
      document.getElementById('speedVal').textContent=spd.toFixed(1);
      document.getElementById('speedVal').classList.toggle('active',spd>0.5&&active==='true');
      var arc=Math.min(spd/50,1)*471;
      document.getElementById('ringProgress').style.strokeDasharray=arc+' '+(628-arc);

      // max / avg
      if(spd>maxSpeed) maxSpeed=spd;
      document.getElementById('maxSpeed').textContent=maxSpeed.toFixed(1);
      speedSum+=spd; speedSamples++;
      var avgCalc=speedSamples>0?speedSum/speedSamples:0;
      document.getElementById('avgSpeedTop').textContent=avgCalc.toFixed(1);

      // incline
      document.getElementById('inclineVal').innerHTML=inc.toFixed(1)+'&deg;';
      document.getElementById('inclineFill').style.height=Math.min(Math.abs(inc)/20*100,100)+'%';

      // stats
      document.getElementById('distVal').textContent=dist.toFixed(2);
      document.getElementById('calVal').textContent=Math.floor(cal);
      document.getElementById('magnetVal').textContent=magnet;

      // elevation (rough estimate from incline * distance delta)
      if(active==='true'&&inc>0.5) elevGained+=0.005;
      document.getElementById('elevVal').textContent=Math.floor(elevGained);

      // duration — use ESP32 value directly
      document.getElementById('durationVal').textContent=fmt(dur);
      document.getElementById('tripStatus').textContent=active==='true'?'Trip active':'Not started';

      // sync badge
      var badge=document.getElementById('syncBadge');
      badge.textContent=fb.toUpperCase();
      badge.className='sync-badge '+(fb==='Connected'?'online':'offline');
      document.getElementById('syncSub').textContent=
        (lastUpd==='Never')?'Last update: never':'Last: '+lastUpd;

      // sparkline
      sparkHistory.push(spd);
      if(sparkHistory.length>MAX_SPARK) sparkHistory.shift();
      drawSparkline();
    }).catch(function(){});
  }, 500);
</script>
</body>
</html>
)rawhtml";

// ─────────────────────────────────────────────────────────────
// ISR
// ─────────────────────────────────────────────────────────────
void IRAM_ATTR onMagnetDetect() {
  unsigned long now = millis();
  unsigned long timeElapsed = now - lastMagnetTime;
  if (timeElapsed > 20) {
    hallSpeedKph = ((float)WHEEL_CIRCUMFERENCE_MM / (float)timeElapsed) * 3.6;
    lastMagnetTime = now;
    rotationCount++;
  }
}

// ─────────────────────────────────────────────────────────────
// HELPERS
// ─────────────────────────────────────────────────────────────
float calculateMET(float speed, float incline) {
  float met = 1.0;
  if      (speed < 1.0)  met = 1.0;
  else if (speed < 16.0) met = 4.0;
  else if (speed < 19.0) met = 6.8;
  else if (speed < 22.0) met = 8.0;
  else                   met = 10.0;
  if (incline > 0) met += (incline / 5.0) * 0.5;
  return met;
}

void pushPendingBatch(float *s, float *i, float *c) {
  if (pendingCount >= MAX_PENDING) {
    Serial.println("Pending queue full. Dropping batch.");
    return;
  }
  Batch &b = pendingBatches[pendingTail];
  for (int k = 0; k < BUFFER_SIZE; ++k) {
    b.speed[k]    = s[k];
    b.incline[k]  = i[k];
    b.calories[k] = c[k];
  }
  pendingTail = (pendingTail + 1) % MAX_PENDING;
  pendingCount++;
  Serial.printf("Batch queued. pendingCount=%d\n", pendingCount);
}

bool sendPendingBatch() {
  if (pendingCount == 0) return true;
  Batch &b = pendingBatches[pendingHead];
  FirebaseJson json;
  FirebaseJsonArray speedArr, inclineArr, calArr;
  for (int k = 0; k < BUFFER_SIZE; ++k) {
    speedArr.add(b.speed[k]);
    inclineArr.add(b.incline[k]);
    calArr.add(b.calories[k]);
  }
  json.set("speed",   speedArr);
  json.set("incline", inclineArr);
  json.set("calories",calArr);
  String path = "/batches/" + String(millis());
  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    pendingHead = (pendingHead + 1) % MAX_PENDING;
    pendingCount--;
    return true;
  }
  Serial.println("Batch send failed: " + fbdo.errorReason());
  return false;
}

float totalSpeed() {
  float s = 0;
  for (int i = 0; i < bufferIndex; ++i) s += speedArray[i];
  return s;
}
float totalIncline() {
  float s = 0;
  for (int i = 0; i < bufferIndex; ++i) s += inclineArray[i];
  return s;
}

void updateFirebaseSummary() {
  if (WiFi.status() != WL_CONNECTED || !Firebase.ready()) return;
  String path = "/trips/" + String(tripStartTime);
  FirebaseJson json;
  json.set("startTime",   (int)tripStartTime);
  json.set("duration",    (int)tripDuration);
  json.set("distance",    tripDistance);
  json.set("avgSpeed",    bufferIndex > 0 ? (totalSpeed() / bufferIndex) : 0.0);
  json.set("avgIncline",  bufferIndex > 0 ? (totalIncline() / bufferIndex) : 0.0);
  json.set("calories",    caloriesBurned);
  json.set("currentSpeed",  fusedSpeedKph);
  json.set("currentIncline",zIncline);
  json.set("active",      tripActive);
  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json))
    Serial.println("Firebase summary OK");
  else
    Serial.println("Firebase summary failed: " + fbdo.errorReason());
}

// ─────────────────────────────────────────────────────────────
// HTTP HANDLERS
// ─────────────────────────────────────────────────────────────
void handleRoot() {
  // serve from PROGMEM — safe for large HTML files
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  String lastUpdate = lastFirebaseUpdate > 0
    ? String((millis() - lastFirebaseUpdate) / 1000) + "s ago"
    : "Never";

  String fbStatus = "Offline";
  if (WiFi.status() == WL_CONNECTED)
    fbStatus = Firebase.ready() ? "Connected" : "Connecting...";

  String data =
    String(displaySpeedKph, 1) + "," +
    String(zIncline, 1)        + "," +
    String(tripDuration)       + "," +
    String(tripDistance, 2)    + "," +
    String(bufferIndex > 0 ? (totalSpeed() / bufferIndex) : 0.0, 1) + "," +
    String(caloriesBurned, 1)  + "," +
    fbStatus                   + "," +
    lastUpdate                 + "," +
    magnetState                + "," +
    (tripActive ? "true" : "false");

  server.send(200, "text/plain", data);
}

void handleTrip() {
  if (!server.hasArg("action")) { server.send(400, "text/plain", "Missing action"); return; }
  String action = server.arg("action");

  if (action == "start" && !tripActive) {
    tripActive     = true;
    tripStartTime  = millis();
    tripDuration   = 0;
    tripDistance   = 0.0;
    caloriesBurned = 0.0;
    rotationCount  = 0;
    bufferIndex    = 0;
    pendingHead = pendingTail = pendingCount = 0;
    magnetState    = "Standby";
    server.send(200, "text/plain", "Trip started");

  } else if (action == "stop" && tripActive) {
    tripActive = false;
    if (bufferIndex > 0 && pendingCount < MAX_PENDING) {
      pushPendingBatch(speedArray, inclineArray, caloriesArray);
      bufferIndex = 0;
    }
    while (pendingCount > 0) { if (!sendPendingBatch()) break; }
    updateFirebaseSummary();
    server.send(200, "text/plain", "Trip stopped");

  } else {
    server.send(200, "text/plain", "Invalid action");
  }
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(hallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), onMagnetDetect, FALLING);

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found — halting.");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 OK");

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(500); Serial.print("."); attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi failed — Firebase disabled.");
  }

  if (WiFi.status() == WL_CONNECTED) {
    config.api_key       = FIREBASE_API_KEY;
    config.database_url  = "https://" + String(FIREBASE_HOST);
    config.token_status_callback = tokenStatusCallback;
    if (Firebase.signUp(&config, &auth, "", ""))
      Serial.println("Firebase sign-up OK");
    else
      Serial.printf("Firebase sign-up error: %s\n", config.signer.signupError.message.c_str());
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
  }

  server.on("/",     handleRoot);
  server.on("/data", handleData);
  server.on("/trip", handleTrip);
  server.begin();
  Serial.println("HTTP server started.");

  lastMagnetTime   = millis();
  lastIMUUpdate    = millis();
  lastSampleTime   = millis();
  lastAnimationTime= millis();
}

// ─────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  pitch    = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  roll     = atan2(-a.acceleration.x, a.acceleration.z) * 180.0 / PI;
  zIncline = asin(constrain(a.acceleration.z / 9.81, -1.0, 1.0)) * 180.0 / PI;
  if (isnan(zIncline)) zIncline = 0.0;
  accelY   = a.acceleration.y;

  unsigned long now = millis();
  float dt = (now - lastIMUUpdate) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastIMUUpdate = now;

  imuVelocityMS += accelY * dt;
  imuVelocityMS *= 0.998;
  if (imuVelocityMS < 0) imuVelocityMS = 0.0;
  imuSpeedKph = imuVelocityMS * 3.6;

  float hallLocal;
  noInterrupts();
  hallLocal = hallSpeedKph;
  interrupts();

  if (millis() - lastMagnetTime > SPEED_TIMEOUT_MS) {
    hallLocal = 0.0;
    noInterrupts(); hallSpeedKph = 0.0; interrupts();
    magnetState = "Standby";
  } else {
    magnetState = "DETECTED";
  }

  fusedSpeedKph = (hallLocal + imuSpeedKph) / 2.0;

  if (tripActive) {
    tripDuration = millis() - tripStartTime;
    tripDistance = (rotationCount * WHEEL_CIRCUMFERENCE_MM) / 1000000.0;
    float met = calculateMET(fusedSpeedKph, fabs(zIncline));
    caloriesBurned += met * CYCLIST_WEIGHT_KG * (1.0 / 3600.0) * dt;
  }

  // Animate display speed
  if (millis() - lastAnimationTime >= ANIMATION_STEP_MS) {
    lastAnimationTime = millis();
    float target = fusedSpeedKph;
    if (fabs(displaySpeedKph - target) < 0.25) {
      displaySpeedKph = target;
    } else if (displaySpeedKph < target) {
      displaySpeedKph = min(displaySpeedKph + ANIMATION_STEP_VALUE, target);
    } else {
      displaySpeedKph = max(displaySpeedKph - ANIMATION_STEP_VALUE, target);
    }
  }

  // 1 Hz buffering
  if (millis() - lastSampleTime >= 1000) {
    lastSampleTime += 1000;
    if (!(pendingCount >= MAX_PENDING && bufferIndex == BUFFER_SIZE)) {
      if (bufferIndex < BUFFER_SIZE) {
        speedArray[bufferIndex]    = fusedSpeedKph;
        inclineArray[bufferIndex]  = fabs(zIncline);
        float met = calculateMET(fusedSpeedKph, fabs(zIncline));
        caloriesArray[bufferIndex] = met * CYCLIST_WEIGHT_KG / 3600.0;
        bufferIndex++;
      }
      if (bufferIndex >= BUFFER_SIZE) {
        if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {
          FirebaseJson json;
          FirebaseJsonArray sArr, iArr, cArr;
          for (int k = 0; k < BUFFER_SIZE; ++k) {
            sArr.add(speedArray[k]);
            iArr.add(inclineArray[k]);
            cArr.add(caloriesArray[k]);
          }
          json.set("speed",   sArr);
          json.set("incline", iArr);
          json.set("calories",cArr);
          String path = "/batches/" + String(millis());
          if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
            Serial.println("Direct batch sent.");
            bufferIndex = 0;
            while (pendingCount > 0) { if (!sendPendingBatch()) break; }
          } else {
            pushPendingBatch(speedArray, inclineArray, caloriesArray);
            bufferIndex = 0;
          }
        } else {
          pushPendingBatch(speedArray, inclineArray, caloriesArray);
          bufferIndex = 0;
        }
      }
    }
  }

  // Periodic retry
  if (millis() - lastFirebaseUpdate >= FIREBASE_UPDATE_INTERVAL) {
    lastFirebaseUpdate = millis();
    if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {
      while (pendingCount > 0) { if (!sendPendingBatch()) break; }
    }
  }
}
