#include <Wire.h> 
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <AiEsp32RotaryEncoder.h>
#include <Adafruit_INA219.h>


/////////////////////// Current settings //////////////////////

int currentSetting = 0;
float maxI = 1.0; // Amps
float minU = 3.2; // Volts

/////////////////////// Current readings //////////////////////

float sourceU = 0.0;
float sourceI = 0.0;


/////////////////////// Measurement counters //////////////////

bool isMeasuringCapacity = false;

float measuredCapacityMilliWattHours = 0.0;
float measurementDurationSeconds = 0;


/////////////////////// Beeping /////////////////////////////

#define BUZZER_PIN 27
#define BUZZER_CHANNEL 0

unsigned long beepUntilMillis = 0;

void beepSetup() {
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWrite(BUZZER_CHANNEL, 0);
}

void beep(unsigned int hz, unsigned int durationMilliseconds) {
  ledcSetup(BUZZER_CHANNEL, hz, 8);
  ledcWrite(BUZZER_CHANNEL, 127);
  beepUntilMillis = millis() + durationMilliseconds;
}

void beepUpdate() {
  if (beepUntilMillis > 0 && beepUntilMillis < millis()) {
    ledcWrite(BUZZER_CHANNEL, 0);
    beepUntilMillis = 0;
  }
}


/////////////////////// Start/Stop Button + Rotary encoder ///

#define ROTARY_ENCODER_A_PIN 35
#define ROTARY_ENCODER_B_PIN 33
#define ROTARY_ENCODER_BUTTON_PIN 14
#define ROTARY_ENCODER_STEPS 2

#define START_STOP_BUTTON_PIN 17


AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(
  ROTARY_ENCODER_A_PIN,
  ROTARY_ENCODER_B_PIN,
  ROTARY_ENCODER_BUTTON_PIN,
  -1, // VCC pin if a rotary encoder module is used
  ROTARY_ENCODER_STEPS,
  false // areEncoderPinsPulldownforEsp32
);

void IRAM_ATTR readEncoderISR() {
    rotaryEncoder.readEncoder_ISR();
}


#define DEBOUNCE_MILLIS 500
unsigned long volatile lastStartStopButtonPressTime = 0;
unsigned long volatile lastStartStopButtonReleaseTime = 0;
volatile bool startStopButtonPressed = false;

void IRAM_ATTR startStopButtonPressed_ISR() {

  unsigned long nowMillis = millis();
  bool newIsPressed = !digitalRead(START_STOP_BUTTON_PIN);
  
  if (newIsPressed) {
    unsigned long lastButtonPress = lastStartStopButtonPressTime;
    lastStartStopButtonPressTime = nowMillis;
    if (nowMillis - lastButtonPress < DEBOUNCE_MILLIS) return;
    if (nowMillis - lastStartStopButtonReleaseTime < DEBOUNCE_MILLIS) return;
    
    startStopButtonPressed = true;
  } else {
    lastStartStopButtonReleaseTime = nowMillis;
  }
}

bool wasStartStopPressed() {
  bool retval = startStopButtonPressed;
  startStopButtonPressed = false;
  return retval;
}



/////////////////////// LCD display //////////////////////////

hd44780_I2Cexp lcd(0x3F);


void updateScreen() {

  unsigned long nowMillis = millis();
  char linebuffer[20];
  
  //lcd.clear(); // we'll use spaces instead of clearing to avoid subtle blinks
  lcd.home();
  sprintf(linebuffer, "In: %.2fV %.2fA ", sourceU, sourceI);
  lcd.print(linebuffer);
  
  lcd.setCursor(0, 1);
  sprintf(linebuffer, "Imax: %.1fA      ", maxI);
  lcd.print(linebuffer);

  lcd.setCursor(0, 2);
  sprintf(linebuffer, "Ucut: %.1fV      ", minU);
  lcd.print(linebuffer);
  
  lcd.setCursor(0, 3);

  if ((nowMillis % 6000) < 3000) { // blinking cursor
    
    int measurementSeconds = int(measurementDurationSeconds) % 60;
    int measurementMinutes = int(measurementDurationSeconds / 60) % 60;
    int measurementHours = int(measurementDurationSeconds / 3600);
    
    sprintf(linebuffer, "Time: %02d:%02d:%02d  ", measurementHours, measurementMinutes, measurementSeconds);
    lcd.print(linebuffer);
  } else {
    sprintf(linebuffer, " %6.1f mWh   ", measuredCapacityMilliWattHours);
    lcd.print(linebuffer);
  }

  if ((nowMillis % 1000) < 500) { // blinking cursor
    lcd.setCursor(13, 1+currentSetting);
    lcd.print("<<<");
  }

  if (isMeasuringCapacity) {
    lcd.setCursor(15, 4);
    
    if ((nowMillis % 2000) < 500) { // spinner
      lcd.print('|');
    } else if ((nowMillis % 2000) < 1000) {
      lcd.print('/');
    } else if ((nowMillis % 2000) < 1500) {
      lcd.print('-');
    } else {
      lcd.print('/');
    }
  }
}

void lcdHalt(const char* msg) {
  lcd.home();
  lcd.print(msg);
}


/////////////////////// Settings input ///////////////////////

void settingsSelect(int index) {
  currentSetting = index;

  if (currentSetting == 0) { // set maxI
    rotaryEncoder.setBoundaries(1, 30, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.setEncoderValue(maxI * 10);
    rotaryEncoder.disableAcceleration();
  } else { // set minU
    rotaryEncoder.setBoundaries(1, 200, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.setAcceleration(10);
    rotaryEncoder.setEncoderValue(minU * 10);
  }
  
  beep(1500, 40);
  updateScreen();
}

void settingsUpdate() {
    if (currentSetting == 0) { // set maxI
    maxI = rotaryEncoder.readEncoder() / 10.0;
  } else { // set minU
    minU = rotaryEncoder.readEncoder() / 10.0;
  }
  
  beep(1000, 20);
  updateScreen();
}



/////////////////////// Current control //////////////////////

#define CURRENT_SET_DAC_PIN 25

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setDrawCurrent(float amps) {
    int dacValue = floor(mapfloat(amps, 0, 3.3, 0, 255)); // [0, 255] range
    dacWrite(CURRENT_SET_DAC_PIN, dacValue);
}


/////////////////////// Measurement Procedures ///////////////

#define MEASUREMENT_INTERVAL_MILLIS 500

Adafruit_INA219 currentSensor;


unsigned long nextMeasurementAtMillis = 0;



void capacityMeasurementsUpdate() {
  
  if (isMeasuringCapacity) {
    setDrawCurrent(maxI);
  } else {
    setDrawCurrent(0);
  }

  if (nextMeasurementAtMillis > millis()) {
    return;
  }
  
  sourceU = currentSensor.getBusVoltage_V();
  sourceI = currentSensor.getCurrent_mA() / 1000.0;
  if (sourceI < 0.0 && sourceI > -0.5) sourceI = 0.0; 
  
  float power_mW = currentSensor.getPower_mW();

  
  if (isMeasuringCapacity) {
    float measurementSeconds = MEASUREMENT_INTERVAL_MILLIS / 1000.0;
    measuredCapacityMilliWattHours += power_mW * measurementSeconds / 3600.0;
    measurementDurationSeconds += measurementSeconds;

    // If load voltage is under cutoff threshold, turn off
    if (sourceU < minU) {
      beep(1750, 1000);
      setDrawCurrent(0);
      isMeasuringCapacity = false;
    }
  }
  
  updateScreen();
  nextMeasurementAtMillis += MEASUREMENT_INTERVAL_MILLIS;
}



/////////////////////// Arduino procedures ///////////////////

void setup() {
  Serial.begin(115200);

  setDrawCurrent(0);
  
  lcd.begin(16, 4);
  lcd.backlight();
  
  if (!currentSensor.begin()) {
    Serial.println("HALT: Failed to find INA219 measurement chip detected on I2C bus");
    lcdHalt("No measurement chip on I2C");
    while (1) { delay(10); }
  }

  pinMode(START_STOP_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(START_STOP_BUTTON_PIN, startStopButtonPressed_ISR, CHANGE);
  
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  settingsSelect(0);

  beepSetup();

}


void loop() {
  if (rotaryEncoder.encoderChanged()) {
    settingsUpdate();
  }
  
  if (rotaryEncoder.isEncoderButtonClicked()) {
    int newSetting = (currentSetting + 1) % 2;
    settingsSelect(newSetting);
  }

  if (wasStartStopPressed()) {
    isMeasuringCapacity = !isMeasuringCapacity;
    beep(1650, 800);
  }
  capacityMeasurementsUpdate();

  beepUpdate();


}
