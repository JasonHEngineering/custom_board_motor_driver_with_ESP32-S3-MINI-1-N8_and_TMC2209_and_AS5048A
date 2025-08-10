// V1 

#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>

/*PINS
   Arduino SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = 10

   STM32 SPI pins
   MOSI = PA7, MISO = PA6, SCK = PA5, CS = PA4

   ESP32_S3_DevKit_C1_N16R8 SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = per assignment as needed

   ESP32-S3-MINI-1-N8 custom board SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = per assignment as needed
*/

// SPI lines for my custom ESP32-S3-MINI-1-N8
#define SPI_CS 38
#define SPI_MISO 37
#define SPI_SCLK 36
#define SPI_MOSI 35

#define no_of_steppers 1

// ESP32-2 MAC Address: C0:4E:30:90:5B:5C
uint8_t slave1[] = {0xC0, 0x4E, 0x30, 0x90, 0xF2, 0x34};

float axis_degAngle[no_of_steppers] = {0}; //Angle in degrees
float accepted_axis_degAngle[no_of_steppers] = {0}; //Angle in degrees
const int windowSize = 10; // Adjust the window size based on your requirements
int currentIndex = 0;
uint16_t rawData = 0; //bits from the encoder (16 bits, but top 2 has to be discarded)
uint16_t command = 0b1111111111111111; //read command (0xFFF)
float sensorValues[no_of_steppers][windowSize];
const float start_axis_degAngle[no_of_steppers] = {177.74}; //Angle in degrees, encoder values when in kinematic zeroed position - 19-Apr-25

// float struct to store angles 
struct encoder {
   float stepper_angle[no_of_steppers]; // unit: °
}; 

encoder outgoingMessage = {0.0};


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.print("📤 Sent to: ");
  Serial.print(macStr);
  Serial.print(" | Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void addPeer(uint8_t *mac) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_add_peer(&peerInfo);
  }
}


void setup() {

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI); //SCLK, MISO, MOSI, SS
  pinMode(SPI_CS, OUTPUT); // set all CS pin as output
  digitalWrite(SPI_CS, HIGH);
  Serial.begin(9600); // start serial for output
  delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  addPeer(slave1);


}

void loop() {
  // put your main code here, to run repeatedly:
  read_angle_Register(); //read the position of the magnet
  report_angle();

  //outgoingMessage.stepper_angle = normalizeAngle(outgoingMessage.stepper_angle + 30.0);
  esp_now_send(slave1, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));
  delay(10); // Give time for callback

}

void read_angle_Register()
{
  for (int i = 0; i <= (no_of_steppers-1); i++) {
    
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    //SPI.setClockDivider(SPI_CLOCK_DIV64);
      
    //--sending the command
    digitalWrite(SPI_CS, LOW);
    SPI.transfer16(command);
    digitalWrite(SPI_CS, HIGH);
  
    //--receiving the reading
    digitalWrite(SPI_CS, LOW);
    rawData = SPI.transfer16(command);
    digitalWrite(SPI_CS, HIGH);
    
    SPI.endTransaction();
  
    rawData = rawData & 0b0011111111111111; //removing the top 2 bits (PAR and EF)
  
    axis_degAngle[i] = (float)rawData / 16383.0 * 360.0; //16384 = 2^14, 360 = 360 degrees, however for real resolution, take 2^14 - 1 = 16383
    updateMovingAverage(i, axis_degAngle[i]);

    // Note: if getMovingAverage(i) == 360.00, it is likely to be false/no reading
    if ((axis_degAngle[i] != 0.00) && (getMovingAverage(i) != 0.00)){
      accepted_axis_degAngle[i] = getMovingAverage(i);
      outgoingMessage.stepper_angle[i] = fmod(((accepted_axis_degAngle[i]-start_axis_degAngle[i])+180.0),360.0)-180.0;

    }
  }
}

void updateMovingAverage(int axis, float newValue) {
  // Update the array with the new value
  sensorValues[axis][currentIndex] = newValue;

  // Move to the next index in a circular manner
  if (axis==(no_of_steppers-1)){
    currentIndex = (currentIndex + 1) % windowSize;
  }
}


float getMovingAverage(int axis) {
  // Calculate the average of the values in the array
  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += sensorValues[axis][i];
  }
  float local_window = windowSize;
  float moving_average = sum / local_window;
  return moving_average;
}



void report_angle()
{
  for(int j = 0; j <= (no_of_steppers-1); j++) {
      //Serial.println(String(j+1) + " " + getMovingAverage(j));
      Serial.println(String("Angle: ") + getMovingAverage(j));
      //Serial.println(String(j+1) + " " + encoder.angles_array[j]);
    }
    Serial.flush();
}

