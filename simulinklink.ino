//#include <Wire.h>

//#define DEBUG

uint8_t incomingByte = 0; // for incoming serial data

#define INTERVAL_MS 100
#define CURRENT_CYCLE (millis() / INTERVAL_MS)

#define OUT_PIN 0
#define CHANNEL_COUNT 8

uint16_t outChannels[CHANNEL_COUNT];
uint16_t inChannels[CHANNEL_COUNT];

bool outChannelsUpdate[CHANNEL_COUNT];

void setup() {
  // put your setup code here, to run once:
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    outChannels[i] = 0;
    inChannels[i] = 0;
    outChannelsUpdate[i] = false;
  }

  adcdac_setup();
  
  Serial.begin(115200);

//  pinMode(OUT_PIN,OUTPUT);

//  digitalWrite(OUT_PIN,HIGH);

//  Wire.begin(); // join i2c bus (address optional for master)
}

bool out = false;

uint32_t cycle = 0;

void loop() {
  // wait
  do {
    // read serial
    while (Serial.available() > 0) {

      // determine channel
      uint8_t outChannel = Serial.read();
  
      outChannelsUpdate[outChannel] = true;
      
      // read 16bit val
      outChannels[outChannel] += Serial.read();
      outChannels[outChannel] = Serial.read() << 8;
  
      Serial.read();
    }
    // write dac
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
      if (!outChannelsUpdate[i]) continue;
      outChannelsUpdate[i] = false;
      dacSetCode(i, outChannels[i]);
    }
  } while (CURRENT_CYCLE == cycle);
  cycle = CURRENT_CYCLE;
  // read adc

  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    inChannels[i] = adcReadRaw(i);
  }

  // write serial
  Serial.write(0);
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    #ifdef DEBUG
    Serial.print("CH ");
    Serial.print(i);
    Serial.print(" VAL ");
    Serial.println(inChannels[i]);
    #else
    Serial.write(inChannels[i] & 0xFF);
    Serial.write(inChannels[i] >> 8);
    #endif
  }
  Serial.write('\n');
  
  Serial.flush();  
}
