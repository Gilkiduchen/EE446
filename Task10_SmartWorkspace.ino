#include <PDM.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_APDS9960.h>
#include <math.h>

// ---------- Microphone (PDM) ----------
short sampleBuffer[256];
volatile int samplesRead = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable > (int)sizeof(sampleBuffer)) {
    bytesAvailable = sizeof(sampleBuffer);
  }

  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;  // 2 bytes per 16-bit sample
}

// ---------- Thresholds----------
const int MIC_THRESHOLD = 200;            
const int CLEAR_DARK_THRESHOLD = 100;      
const float MOTION_THRESHOLD = 25.0f;      
const int PROX_NEAR_THRESHOLD = 100;       

// ---------- Update period ----------
const unsigned long UPDATE_MS = 200;
unsigned long lastUpdateMs = 0;

// ---------- Latest raw values ----------
int micLevel = 0;      // microphone activity level
int clearValue = 0;    // APDS clear channel
float motionValue = 0; // IMU motion metric (gyro magnitude)
int proxValue = 0;     // APDS proximity

struct SituationProfile {
  const char* label;
  uint8_t sound;
  uint8_t dark;
  uint8_t moving;
  uint8_t near;
};

const SituationProfile profiles[] = {
  {"QUIET_BRIGHT_STEADY_FAR", 0, 0, 0, 0},
  {"NOISY_BRIGHT_STEADY_FAR", 1, 0, 0, 0},
  {"QUIET_DARK_STEADY_NEAR", 0, 1, 0, 1},
  {"NOISY_BRIGHT_MOVING_NEAR", 1, 0, 1, 1},
};

const int NUM_PROFILES = sizeof(profiles) / sizeof(profiles[0]);

const char* classifyState(uint8_t sound, uint8_t dark, uint8_t moving, uint8_t near) {
  // Rule-based nearest prototype (minimum Hamming distance).
  int bestIdx = 0;
  int bestDist = 99;

  for (int i = 0; i < NUM_PROFILES; i++) {
    int dist = 0;
    dist += (sound != profiles[i].sound);
    dist += (dark != profiles[i].dark);
    dist += (moving != profiles[i].moving);
    dist += (near != profiles[i].near);

    if (dist < bestDist) {
      bestDist = dist;
      bestIdx = i;
    }
  }

  return profiles[bestIdx].label;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1) {}
  }

  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960.");
    while (1) {}
  }

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM microphone.");
    while (1) {}
  }

  Serial.println("Task 10 classifier started.");
}

void loop() {
  // ----- Mic level update -----
  if (samplesRead > 0) {
    long sumAbs = 0;
    int n = samplesRead;
    for (int i = 0; i < n; i++) {
      sumAbs += abs(sampleBuffer[i]);
    }
    micLevel = (n > 0) ? (int)(sumAbs / n) : 0;
    samplesRead = 0;
  }

  // ----- Ambient light (clear channel) update -----
  if (APDS.colorAvailable()) {
    int r, g, b, c;
    APDS.readColor(r, g, b, c);
    clearValue = c;
  }

  // ----- Proximity update -----
  if (APDS.proximityAvailable()) {
    proxValue = APDS.readProximity();
  }

  // ----- IMU motion metric update (gyro magnitude) -----
  if (IMU.gyroscopeAvailable()) {
    float gx, gy, gz;
    IMU.readGyroscope(gx, gy, gz);
    motionValue = sqrt(gx * gx + gy * gy + gz * gz);
  }

  unsigned long now = millis();
  if (now - lastUpdateMs < UPDATE_MS) {
    return;
  }
  lastUpdateMs = now;

  // Binary decisions for each sensing modality
  uint8_t sound = (micLevel >= MIC_THRESHOLD) ? 1 : 0;
  uint8_t dark = (clearValue <= CLEAR_DARK_THRESHOLD) ? 1 : 0;
  uint8_t moving = (motionValue >= MOTION_THRESHOLD) ? 1 : 0;
  uint8_t near = (proxValue >= PROX_NEAR_THRESHOLD) ? 1 : 0;

  const char* label = classifyState(sound, dark, moving, near);

  // Output format
  Serial.print("raw,mic=");
  Serial.print(micLevel);
  Serial.print(",clear=");
  Serial.print(clearValue);
  Serial.print(",motion=");
  Serial.print(motionValue, 3);
  Serial.print(",prox=");
  Serial.println(proxValue);

  Serial.print("flags,sound=");
  Serial.print(sound);
  Serial.print(",dark=");
  Serial.print(dark);
  Serial.print(",moving=");
  Serial.print(moving);
  Serial.print(",near=");
  Serial.println(near);

  Serial.print("state,");
  Serial.println(label);
}
