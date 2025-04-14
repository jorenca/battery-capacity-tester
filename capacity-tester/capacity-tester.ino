#include <Wire.h> 
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <AiEsp32RotaryEncoder.h>
#include <ToneESP32.h>


/////////////////////// Current settings //////////////////////

int currentSetting = 0;
float maxI = 1.0; // Amps
float minU = 3.2; // Volts

/////////////////////// Current readings //////////////////////

float sourceU = 0.0;
float sourceI = 0.0;


/////////////////////// Beeping /////////////////////////////

#define BUZZER_PIN 27
#define BUZZER_CHANNEL 0

ToneESP32 buzzer(BUZZER_PIN, BUZZER_CHANNEL);


void beep(unsigned int hz, unsigned int milliseconds) {
  buzzer.tone(hz, milliseconds);
}


/////////////////////// Rotary encoder ///////////////////////

#define ROTARY_ENCODER_A_PIN 35
#define ROTARY_ENCODER_B_PIN 33
#define ROTARY_ENCODER_BUTTON_PIN 14
#define ROTARY_ENCODER_STEPS 2

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


/////////////////////// LCD display //////////////////////////

hd44780_I2Cexp lcd(0x3F);


void updateScreen() {
  // TODO CONSIDER REWRITING USING sprintf
  
  //lcd.clear(); // we'll use spaces instead of clearing to avoid subtle blinks
  lcd.home();
  lcd.print("In: ");
  lcd.print(sourceU);
  lcd.print("V ");
  lcd.print(sourceI);
  lcd.print('A');
  
  lcd.print("   "); // avoids issues with newline
  lcd.setCursor(0, 1);

  lcd.print("Imax: ");
  lcd.print(maxI);
  lcd.print('A');
  
  lcd.print("      "); // avoids issues with newline
  lcd.setCursor(0, 2);

  lcd.print("Ucut: ");
  lcd.print(minU);
  lcd.print('V');
  
  lcd.print("      "); // avoids issues with newline
  lcd.setCursor(0, 3);

  // TODO Capacity: 10000mWh / Time: 00:00:00

  lcd.setCursor(13, 1+currentSetting);
  lcd.print("<<<");
}


/////////////////////// LCD display //////////////////////////

void settingsSelect(int index) {
  currentSetting = index;

  if (currentSetting == 0) { // set maxI
    rotaryEncoder.setBoundaries(1, 33, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.setEncoderValue(maxI * 10);
    rotaryEncoder.disableAcceleration();
  } else { // set minU
    rotaryEncoder.setBoundaries(1, 300, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
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


/////////////////////// Arduino procedures ///////////////////

void setup() {
  Serial.begin(115200);
  
  lcd.begin(16, 4);
  lcd.backlight();
  
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  settingsSelect(0);


}


void loop() {
  if (rotaryEncoder.encoderChanged()) {
    settingsUpdate();
  }
  
  if (rotaryEncoder.isEncoderButtonClicked()) {
    int newSetting = (currentSetting + 1) % 2;
    settingsSelect(newSetting);
  }

}
