// #include <SoftwareSerial.h>
// SoftwareSerial mspSerial(A5, A6); // RX TX
// #include <HardwareSerial.h>
// HardwareSerial mspSerial(1);
#define mspSerial (Serial1)

// 0x1F01 = 7937 - Rangefinder data
#define MSP_SENSOR_RANGEFINDER (0x1F01)
// 0x1F02 = 7938 - Optical flow data
#define MSP_SENSOR_OPTICAL_FLOW (0x1F02)
struct __attribute__((packed)) mspSensorOpticalFlowDataMessage_t {
  uint8_t quality;    // [0;255]
  int32_t motionX;
  int32_t motionY;
};
struct __attribute__((packed)) mspSensorRangefinderDataMessage_t {
  uint8_t quality;    // [0;255]
  int32_t distanceMm; // Negative value for out of range
};

// https://github.com/iNavFlight/inav/wiki/MSP-V2#crc_dvb_s2-example
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
  crc ^= a;
  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0xD5;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  mspSerial.begin(115200);
  
  while (!Serial1) {}
  while (!Serial) {}
}

void readMSPMessage(void (*callback)(uint16_t commandID, uint16_t payloadSize, uint8_t* data)) {
  // https://github.com/iNavFlight/inav/wiki/MSP-V2#msp-v2-message-structure
  uint8_t flag = mspSerial.read();
  
  delay(1);

  uint16_t commandID;
  mspSerial.readBytes((char*)&commandID, 2);

  delay(1);

  uint16_t payloadSize;
  mspSerial.readBytes((char*)&payloadSize, 2);

  delay(1);

  uint8_t expectedChecksum = crc8_dvb_s2(0, flag);
  expectedChecksum = crc8_dvb_s2(expectedChecksum, commandID & 0xFF);
  expectedChecksum = crc8_dvb_s2(expectedChecksum, (commandID >> 8) & 0xFF);
  expectedChecksum = crc8_dvb_s2(expectedChecksum, payloadSize & 0xFF);
  expectedChecksum = crc8_dvb_s2(expectedChecksum, (payloadSize >> 8) & 0xFF);

  uint8_t data[payloadSize];
  if(payloadSize > 0 && mspSerial.available() >= payloadSize) {
    mspSerial.readBytes(data, payloadSize);
    for(int i = 0; i < payloadSize; i++) {
      expectedChecksum = crc8_dvb_s2(expectedChecksum, data[i]);
    }
  }

  delay(1);
  
  while(mspSerial.available() < 1) {
    delay(1);
  }

  uint8_t checksum = mspSerial.read();
  if(checksum == expectedChecksum) {
    callback(commandID, payloadSize, data);
  } else {
    Serial.println("UNEXPECTED VALUE: Checksum mismatch");
  }
}

void handleMSPUpdate(uint16_t commandID, uint16_t payloadSize, uint8_t* data) {
  // This is a really hacky "protocol" to send the data, but it works for now lol
  if(commandID == MSP_SENSOR_OPTICAL_FLOW && payloadSize == sizeof(mspSensorOpflowDataMessage_t)) {
    mspSensorOpflowDataMessage_t* msg = reinterpret_cast<mspSensorOpflowDataMessage_t*>(data);
    Serial.print("### OPTICAL_FLOW | MOTION_X ");
    Serial.print(msg->motionX);
    Serial.print(" | MOTION_Y ");
    Serial.print(msg->motionY);
    Serial.print(" | QUALITY ");
    Serial.println(msg->quality);
  } else if(commandID == MSP_SENSOR_RANGEFINDER && payloadSize == sizeof(mspSensorRangefinderDataMessage_t)) {
    mspSensorRangefinderDataMessage_t* msg = reinterpret_cast<mspSensorRangefinderDataMessage_t*>(data);
    Serial.print("### DISTANCE | DISTANCE ");
    Serial.print(msg->distanceMm);
    Serial.print(" | QUALITY ");
    Serial.println(msg->quality);
  } else {
    Serial.print("UNEXPECTED VALUE: Unknown command id ");
    Serial.println(commandID);
  }
  delay(1);
}

void handleMSPError(uint16_t commandID, uint16_t payloadSize, uint8_t* data) {
  Serial.print("Command ");
  Serial.print(commandID);
  Serial.print(" returned error ");
  if(payloadSize > 0) {
    Serial.print(" with payload: ");
    for(int i = 0; i < payloadSize; i++) {
      Serial.print(data[i]);
      Serial.print(" ");
    }
  }
}

void readMSPHeader() {
  char header[3];
  mspSerial.readBytes(header, 3);

  delay(1);
  
  // https://github.com/iNavFlight/inav/wiki/MSP-V2#message-types
  if (header[0] == '$' && header[1] == 'X') {
    switch(header[2]) {
      case '<':
        readMSPMessage(&handleMSPUpdate);
        break;
      case '!':
        Serial.print("RECIEVED ERROR: ");
        readMSPMessage(&handleMSPError);
        break;
      case '>':
        Serial.println("UNEXPECTED VALUE: Response message (Header: $X>)");
        mspSerial.flush(); // Skip the rest of the message
        break;
      default:
        Serial.println("UNEXPECTED VALUE: Unknown message (Header: $X?)");
        mspSerial.flush(); // Skip the rest of the message
        break;
    }
  }
}

void loop() {
  // Read the MSP header
  if (mspSerial.available() >= 6) {
    readMSPHeader();
  } else {
    delay(1);
  }
}