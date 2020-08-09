#include <SPI.h>
#include <Adafruit_PCD8544.h>
#include <Wire.h>
//#include <Sodaq_SHT2x.h>

// Hardware SPI (faster, but must use certain hardware pins):
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
// Note with hardware SPI MISO and SS pins aren't used but will still be read
// and written to during SPI transfer.  Be careful sharing these pins!
Adafruit_PCD8544 lcd = Adafruit_PCD8544(5, 3, 4);
long lastScreenRefresh = 0;
#define PIN_LCD_LED 10

#define PIN_BATT_VOLT A2
#define PIN_BATT_AMP A1

#define PIN_MOT_PWD 6
#define PIN_MOT_FWD_ENABLE 8
#define PIN_MOT_BACK_ENABLE 7

#define JOYSTICK_MID 128

// populated in the SerialControl part
uint8_t joystickX = 0, joystickY = 0;


#define CURRENT_LIMIT 10 // current in amps before we start forcing the PWM duty cycle down to reduce the current withing an acceptable limit
uint8_t currentLimitThrotling = 0;

int speed = 0;

void setup() {
  
  pinMode(PIN_MOT_FWD_ENABLE, OUTPUT);
  pinMode(PIN_MOT_BACK_ENABLE, OUTPUT);
  pinMode(PIN_MOT_PWD, OUTPUT);

  digitalWrite(PIN_MOT_FWD_ENABLE, LOW);
  digitalWrite(PIN_MOT_BACK_ENABLE, LOW);
  analogWrite(PIN_MOT_PWD, 0);

  // used by the Serial/Joystick control
  //Serial.begin(115200);

  pinMode(PIN_LCD_LED, OUTPUT);
  analogWrite(PIN_LCD_LED, 0);  // [0..255] for backlight intensity
  lcd.begin();
  lcd.setContrast(60);
  lcd.setTextSize(1);
  lcd.setTextColor(BLACK);

  setup_serial_control();

}


void loop() {
  serialControlLoop();
  
  lcd.clearDisplay();
  lcd.setCursor(0,0);

  unsigned long currentTime = millis();

  float voltage = batteryVoltage();
  float amps = batteryCurrent();
  if(amps > CURRENT_LIMIT && currentLimitThrotling < 255) {
    currentLimitThrotling ++;
  } else if (amps < CURRENT_LIMIT && currentLimitThrotling > 0) {
    currentLimitThrotling --;
  }
  
  lcd.print(round(voltage)); lcd.print("V/"); lcd.print(amps, 1); lcd.print("A/"); lcd.print(round(voltage * amps)); lcd.println("W");
  lcd.print("Joy: "); lcd.println(joystickY);
  
  if(isValidJoystickValue(joystickY) && voltage > 20) {
    if(joystickY >= JOYSTICK_MID) {
      digitalWrite(PIN_MOT_FWD_ENABLE, HIGH);
      digitalWrite(PIN_MOT_BACK_ENABLE, LOW);
    } else {
      digitalWrite(PIN_MOT_FWD_ENABLE, LOW);
      digitalWrite(PIN_MOT_BACK_ENABLE, HIGH);
    }

    speed = constrain(round(abs(joystickY - JOYSTICK_MID) * 2.55) - currentLimitThrotling, 0, 255);
  } else {
    speed = 0;
  }

  
  lcd.print("Lmt "); lcd.print(currentLimitThrotling);lcd.print("/S "); lcd.println(speed);
  analogWrite(PIN_MOT_PWD, speed);
  


  // throttle the screen refresh rate to avoid flickering
  if(currentTime - lastScreenRefresh > 200) {
    lcd.display();
    lastScreenRefresh = currentTime;
  }
}



float batteryVoltage() {
  // analogRead -> [0 .. 1023] -> [0..3.3]Volts -> multiply by 10.9 for the actual voltage
  return analogRead(PIN_BATT_VOLT) / 28.2; // factor is ever so slightly different from the above theoretical one after empirical measurements
}

// filter/smooth the readings are there's too much noise
float averagePinBattAmp = 2.5 / 3.3 * 1023;
float batteryCurrent() {
  // the ACS712 20Amps sensor is connected to 5V, so 0 current -> 2.5V 
  // it's wired such that the more current there is the LESS voltage the sensor outputs 
  // with a sensitivity or 100mV / Ampère
  // analogRead -> [0 .. 1023] -> [0..3.3]Volts. Amps = (2500 - analogValue / 1023 * 3300) / 100 
  averagePinBattAmp = averagePinBattAmp * 0.95 + analogRead(PIN_BATT_AMP) * 0.05;
  // the no current value/middle is not exactly 2.5V
  return 25.4 - averagePinBattAmp / 1023 * 33; 
}
