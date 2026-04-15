#include <Arduino_HS300x.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_APDS9960.h>
#include <math.h>
#include <string.h>

// ----------------------------- Thresholds -----------------------------
float RH_JUMP_THRESHOLD = 4.0f;         // %RH rise from baseline
float TEMP_RISE_THRESHOLD = 0.8f;       // deg C rise from baseline
float MAG_SHIFT_THRESHOLD = 12.0f;      // magnetic metric shift from baseline
int CLEAR_CHANGE_THRESHOLD = 120;       // clear channel absolute change
float COLOR_RATIO_THRESHOLD = 0.12f;    // normalized RGB ratio shift
const int CALIBRATION_SAMPLES = 24;
const unsigned long CALIBRATION_SAMPLE_DELAY_MS = 20;

// ----------------------------- Timing / filtering -----------------------------
const unsigned long UPDATE_MS = 250;
const unsigned long EVENT_COOLDOWN_MS = 1500;   // anti rapid retrigger
const int EVENT_DEBOUNCE_COUNT = 2;             // consecutive frames needed
const float BASELINE_ALPHA = 0.03f;             // slow baseline update

unsigned long lastUpdateMs = 0;

// ----------------------------- Raw sensor values -----------------------------
float rhValue = 0.0f;
float tempValue = 0.0f;
float magValue = 0.0f;  // chosen magnetic metric: |B|
int rValue = 0;
int gValue = 0;
int bValue = 0;
int clearValue = 0;

// ----------------------------- Baselines -----------------------------
bool baselineReady = false;
float rhBase = 0.0f;
float tempBase = 0.0f;
float magBase = 0.0f;
float clearBase = 0.0f;
float rNormBase = 0.0f;
float gNormBase = 0.0f;
float bNormBase = 0.0f;

// ----------------------------- Event state -----------------------------
const char* EVENT_BASELINE = "BASELINE_NORMAL";
const char* EVENT_BREATH = "BREATH_OR_WARM_AIR_EVENT";
const char* EVENT_MAG = "MAGNETIC_DISTURBANCE_EVENT";
const char* EVENT_LIGHT = "LIGHT_OR_COLOR_CHANGE_EVENT";

const char* lastCandidate = "BASELINE_NORMAL";
int candidateCount = 0;
const char* lastTriggeredEvent = "BASELINE_NORMAL";
unsigned long lastTriggerMs = 0;

bool isSameLabel(const char* a, const char* b) {
  return strcmp(a, b) == 0;
}

void calibrateThresholds() {
  rhValue = HS300x.readHumidity();
  tempValue = HS300x.readTemperature();

  if (IMU.magneticFieldAvailable()) {
    float mx, my, mz;
    IMU.readMagneticField(mx, my, mz);
    magValue = sqrt(mx * mx + my * my + mz * mz);
  } else {
    magValue = 0.0f;
  }

  if (APDS.colorAvailable()) {
    APDS.readColor(rValue, gValue, bValue, clearValue);
  } else {
    rValue = 0;
    gValue = 0;
    bValue = 0;
    clearValue = 0;
  }

  float sumRgb = (float)rValue + (float)gValue + (float)bValue;
  if (sumRgb < 1.0f) {
    sumRgb = 1.0f;
  }
  float rn = (float)rValue / sumRgb;
  float gn = (float)gValue / sumRgb;
  float bn = (float)bValue / sumRgb;

  float rhMin = rhValue, rhMax = rhValue;
  float tempMin = tempValue, tempMax = tempValue;
  float magMin = magValue, magMax = magValue;
  float clearMin = (float)clearValue, clearMax = (float)clearValue;
  float rNormMin = rn, rNormMax = rn;
  float gNormMin = gn, gNormMax = gn;
  float bNormMin = bn, bNormMax = bn;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    delay(CALIBRATION_SAMPLE_DELAY_MS);

    rhValue = HS300x.readHumidity();
    tempValue = HS300x.readTemperature();

    if (IMU.magneticFieldAvailable()) {
      float mx, my, mz;
      IMU.readMagneticField(mx, my, mz);
      magValue = sqrt(mx * mx + my * my + mz * mz);
    }

    if (APDS.colorAvailable()) {
      APDS.readColor(rValue, gValue, bValue, clearValue);
    }

    sumRgb = (float)rValue + (float)gValue + (float)bValue;
    if (sumRgb < 1.0f) {
      sumRgb = 1.0f;
    }
    rn = (float)rValue / sumRgb;
    gn = (float)gValue / sumRgb;
    bn = (float)bValue / sumRgb;

    rhMin = fmin(rhMin, rhValue);
    rhMax = fmax(rhMax, rhValue);
    tempMin = fmin(tempMin, tempValue);
    tempMax = fmax(tempMax, tempValue);
    magMin = fmin(magMin, magValue);
    magMax = fmax(magMax, magValue);
    clearMin = fmin(clearMin, (float)clearValue);
    clearMax = fmax(clearMax, (float)clearValue);
    rNormMin = fmin(rNormMin, rn);
    rNormMax = fmax(rNormMax, rn);
    gNormMin = fmin(gNormMin, gn);
    gNormMax = fmax(gNormMax, gn);
    bNormMin = fmin(bNormMin, bn);
    bNormMax = fmax(bNormMax, bn);
  }

  float rhSpan = rhMax - rhMin;
  float tempSpan = tempMax - tempMin;
  float magSpan = magMax - magMin;
  float clearSpan = clearMax - clearMin;
  float colorSpan = fmax(rNormMax - rNormMin,
                    fmax(gNormMax - gNormMin, bNormMax - bNormMin));

  RH_JUMP_THRESHOLD = fmin(8.0f, fmax(0.8f, rhSpan * 4.0f));
  TEMP_RISE_THRESHOLD = fmin(2.5f, fmax(0.2f, tempSpan * 4.0f));
  MAG_SHIFT_THRESHOLD = fmin(60.0f, fmax(4.0f, magSpan * 4.0f));
  CLEAR_CHANGE_THRESHOLD = (int)(fmin(1024.0f, fmax(40.0f, clearSpan * 4.0f)) + 0.5f);
  COLOR_RATIO_THRESHOLD = fmin(0.35f, fmax(0.03f, colorSpan * 3.0f));
}

void updateBaseline() {
  rhBase = (1.0f - BASELINE_ALPHA) * rhBase + BASELINE_ALPHA * rhValue;
  tempBase = (1.0f - BASELINE_ALPHA) * tempBase + BASELINE_ALPHA * tempValue;
  magBase = (1.0f - BASELINE_ALPHA) * magBase + BASELINE_ALPHA * magValue;
  clearBase = (1.0f - BASELINE_ALPHA) * clearBase + BASELINE_ALPHA * (float)clearValue;

  float sumRgb = (float)rValue + (float)gValue + (float)bValue;
  if (sumRgb < 1.0f) {
    sumRgb = 1.0f;
  }
  float rn = (float)rValue / sumRgb;
  float gn = (float)gValue / sumRgb;
  float bn = (float)bValue / sumRgb;

  rNormBase = (1.0f - BASELINE_ALPHA) * rNormBase + BASELINE_ALPHA * rn;
  gNormBase = (1.0f - BASELINE_ALPHA) * gNormBase + BASELINE_ALPHA * gn;
  bNormBase = (1.0f - BASELINE_ALPHA) * bNormBase + BASELINE_ALPHA * bn;
}

const char* chooseCandidateEvent(bool humidJump, bool tempRise, bool magShift, bool lightOrColorChange) {
  // Priority rule (deterministic):
  // 1) magnetic disturbance, 2) breath/warm air, 3) light/color change, 4) baseline.
  if (magShift) {
    return EVENT_MAG;
  }
  if (humidJump || tempRise) {
    return EVENT_BREATH;
  }
  if (lightOrColorChange) {
    return EVENT_LIGHT;
  }
  return EVENT_BASELINE;
}

const char* applyDebounceAndCooldown(const char* candidate, unsigned long nowMs) {
  if (isSameLabel(candidate, lastCandidate)) {
    candidateCount++;
  } else {
    lastCandidate = candidate;
    candidateCount = 1;
  }

  // Not stable yet
  if (candidateCount < EVENT_DEBOUNCE_COUNT) {
    return EVENT_BASELINE;
  }

  // Baseline can always pass through.
  if (isSameLabel(candidate, EVENT_BASELINE)) {
    return EVENT_BASELINE;
  }

  // Avoid rapid retrigger of the same event label.
  bool sameAsLastTrigger = isSameLabel(candidate, lastTriggeredEvent);
  bool inCooldown = (nowMs - lastTriggerMs) < EVENT_COOLDOWN_MS;
  if (sameAsLastTrigger && inCooldown) {
    return EVENT_BASELINE;
  }

  lastTriggeredEvent = candidate;
  lastTriggerMs = nowMs;
  return candidate;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize HS300x.");
    while (1) {}
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1) {}
  }

  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960.");
    while (1) {}
  }

  calibrateThresholds();
  Serial.println("Task 11 event monitor started.");
}

void loop() {
  unsigned long now = millis();
  if (now - lastUpdateMs < UPDATE_MS) {
    return;
  }
  lastUpdateMs = now;

  // ----- Read humidity/temperature -----
  rhValue = HS300x.readHumidity();
  tempValue = HS300x.readTemperature();

  // ----- Read magnetometer and compute chosen magnetic metric |B| -----
  if (IMU.magneticFieldAvailable()) {
    float mx, my, mz;
    IMU.readMagneticField(mx, my, mz);
    magValue = sqrt(mx * mx + my * my + mz * mz);
  }

  // ----- Read APDS9960 RGB + clear -----
  if (APDS.colorAvailable()) {
    APDS.readColor(rValue, gValue, bValue, clearValue);
  }

  // ----- Initialize baseline on first valid cycle -----
  if (!baselineReady) {
    rhBase = rhValue;
    tempBase = tempValue;
    magBase = magValue;
    clearBase = (float)clearValue;

    float sumRgb = (float)rValue + (float)gValue + (float)bValue;
    if (sumRgb < 1.0f) {
      sumRgb = 1.0f;
    }
    rNormBase = (float)rValue / sumRgb;
    gNormBase = (float)gValue / sumRgb;
    bNormBase = (float)bValue / sumRgb;

    baselineReady = true;
  }

  // ----- Binary flag decisions -----
  bool humidJump = (rhValue - rhBase) >= RH_JUMP_THRESHOLD;
  bool tempRise = (tempValue - tempBase) >= TEMP_RISE_THRESHOLD;
  bool magShift = fabs(magValue - magBase) >= MAG_SHIFT_THRESHOLD;

  float clearDelta = fabs((float)clearValue - clearBase);
  float sumRgbNow = (float)rValue + (float)gValue + (float)bValue;
  if (sumRgbNow < 1.0f) {
    sumRgbNow = 1.0f;
  }
  float rNormNow = (float)rValue / sumRgbNow;
  float gNormNow = (float)gValue / sumRgbNow;
  float bNormNow = (float)bValue / sumRgbNow;
  float colorShift = fmax(fabs(rNormNow - rNormBase),
                    fmax(fabs(gNormNow - gNormBase), fabs(bNormNow - bNormBase)));

  bool lightOrColorChange = (clearDelta >= (float)CLEAR_CHANGE_THRESHOLD) ||
                            (colorShift >= COLOR_RATIO_THRESHOLD);

  const char* candidate = chooseCandidateEvent(humidJump, tempRise, magShift, lightOrColorChange);
  const char* finalEvent = applyDebounceAndCooldown(candidate, now);

  // Keep baseline adaptive when no active event is being emitted.
  if (isSameLabel(finalEvent, EVENT_BASELINE)) {
    updateBaseline();
  }

  // ----- Required output format (3 lines per update cycle) -----
  Serial.print("raw,rh=");
  Serial.print(rhValue, 2);
  Serial.print(",temp=");
  Serial.print(tempValue, 2);
  Serial.print(",mag=");
  Serial.print(magValue, 3);
  Serial.print(",r=");
  Serial.print(rValue);
  Serial.print(",g=");
  Serial.print(gValue);
  Serial.print(",b=");
  Serial.print(bValue);
  Serial.print(",clear=");
  Serial.println(clearValue);

  Serial.print("flags,humid_jump=");
  Serial.print(humidJump ? 1 : 0);
  Serial.print(",temp_rise=");
  Serial.print(tempRise ? 1 : 0);
  Serial.print(",mag_shift=");
  Serial.print(magShift ? 1 : 0);
  Serial.print(",light_or_color_change=");
  Serial.println(lightOrColorChange ? 1 : 0);

  Serial.print("event,");
  Serial.println(finalEvent);
}
