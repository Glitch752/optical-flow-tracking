// #include <SoftwareSerial.h>
// SoftwareSerial mspSerial(A5, A6); // RX TX
// #include <HardwareSerial.h>
// HardwareSerial mspSerial(1);
#define mspSerial (Serial1)

struct __attribute__((packed)) mspSensorOpflowDataMessage_t {
  uint8_t quality;    // [0;255]
  int32_t motionX;
  int32_t motionY;
};

struct __attribute__((packed)) mspSensorRangefinderDataMessage_t {
  uint8_t quality;    // [0;255]
  int32_t distanceMm; // Negative value for out of range
};

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

// Can return nullptr for error cases!
void readMSPMessage((void (*)(uint16_t, uint16_t, uint8_t*)) callback) {
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


int32_t tempLastDistanceMM = 0;
int32_t tempLastMotionX = 0;
int32_t tempLastMotionY = 0;

void handleMSPUpdate(uint16_t commandID, uint16_t payloadSize, uint8_t* data) {
  if(commandID == 7938 && payloadSize == sizeof(mspSensorOpflowDataMessage_t)) {
    mspSensorOpflowDataMessage_t* msg = reinterpret_cast<mspSensorOpflowDataMessage_t*>(data);
    Serial.print("Motion X: ");
    Serial.print(msg->motionX);
    Serial.print(", Motion Y: ");
    Serial.print(msg->motionY);
    Serial.print(", Quality: ");
    Serial.println(msg->quality);
    // tempLastMotionX = msg->motionX;
    // tempLastMotionY = msg->motionY;
  } else if(commandID == 7937 && payloadSize == sizeof(mspSensorRangefinderDataMessage_t)) {
    mspSensorRangefinderDataMessage_t* msg = reinterpret_cast<mspSensorRangefinderDataMessage_t*>(data);
    // tempLastDistanceMM = msg->distanceMm;
    Serial.print("Distance: ");
    Serial.print(msg->distanceMm);
    Serial.print(", Quality: ");
    Serial.println(msg->quality);
  } else {
    Serial.print("UNEXPECTED VALUE: Unknown command id ");
    Serial.println(commandID);
  }
  delay(1);

  // Serial.print(tempLastMotionX);
  // Serial.print(" ");
  // Serial.print(tempLastMotionY);
  // Serial.print(" ");
  // Serial.print(tempLastDistanceMM);
  // Serial.println();
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
    if(header[2] == '<') {
      // This is the "normal" case
      readMSPMessage(&handleMSPUpdate);
    } else if(header[2] == '>') {
      // Response... it doesn't make sense if we receive this
      Serial.println("UNEXPECTED VALUE: Response message (Header: $X>)");
      mspSerial.flush(); // Skip the rest of the message
    } else if(header[2] == "!") {
      Serial.print("RECIEVED ERROR: ");
      readMSPMessage(&handleMSPError);
    } else {
      Serial.println("UNEXPECTED VALUE: Unknown message (Header: $X?)");
      mspSerial.flush(); // Skip the rest of the message
    }
  }
}

void loop() {
  // Read the MSP header
  if (mspSerial.available() >= 6) {
    readMSPHeader();
  } else {
    // Serial.println("No data");
  }
}