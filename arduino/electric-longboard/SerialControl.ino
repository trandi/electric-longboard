// Using SOFTWARE serial for this induces a noticeable delay (1-200ms) for each iteration plus many transmission errors !
// #include <SoftwareSerial.h>
// SoftwareSerial serCtrl(9, 10); // RX, TX (not used)

/////////////// AUREL Wireless Commands /////////////////////////////////////

uint8_t _prevChar;
uint8_t currChar;
boolean _readingMsg = false;
uint8_t _msg[6];
uint8_t _msgPos;
unsigned long _lastValidDataTime = 0; 


boolean startNewMsg(uint8_t c) {
  boolean res = (_prevChar == 0) && (c == 255);
  _prevChar = c;
  return res;
}


// to be called periodically by the main loop
void serialControlLoop() {
  unsigned long currentTime = millis();


  if(currentTime - _lastValidDataTime > 500) {
    // Signal lost -> going into SAFE mode. Set joystick to INVALID values
    joystickX = 0;
    joystickY = 0;
  }
  
  while (Serial.available()) {
    // keep checking so that we go into safe mode if signal is lost, not wait to read all the queued serial data
    if(currentTime - _lastValidDataTime > 500) {
      // More than 1 sec since last valid control data -> going into SAFE mode
      // set joystick to INVALID values
      joystickX = 0;
      joystickY = 0;
    }
      
    currChar = Serial.read();

    if (startNewMsg(currChar)) {
      _readingMsg = true;
      _msgPos = 0;
    } else if (_readingMsg) {
      if (_msgPos >= 6) {
        // data finished, last byte is the CRC
        uint8_t crc = 0;
        for (uint8_t i = 0; i < 6; i++)
          crc += _msg[i];

        if (crc == currChar) {
          _lastValidDataTime = currentTime; 
                    
          joystickX = _msg[0];
          joystickY = _msg[1];
        } else {
          //Serial.print("Wrong CRC: ");Serial.print(currChar);Serial.print(" Expected: ");Serial.println(crc);
        }

        _readingMsg = false;
      } else {
        // normal data, add it to the message
        _msg[_msgPos++] = currChar;
      }
    }
  }
}

void setup_serial_control() {
  // harware serial of the Arduino connected to the Wireless receiver
  Serial.begin(9600);
  Serial.println("Serial Control set up at 9600bps");
}


boolean isValidJoystickValue(uint8_t joystick) {
  return joystick > 20 && joystick < 230;
}
