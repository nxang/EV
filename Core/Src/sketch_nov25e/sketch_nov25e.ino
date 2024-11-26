#include <SD.h>
#include <EEPROM.h>  // Include the EEPROM library

#define HEADER 0x02
#define FOOTER 0x03

const int chipSelect = 5;
const int BUFFER_SIZE = 128;  // Adjust as needed
char dataBuffer[BUFFER_SIZE];
char received_buffer[50];  // Adjust size as needed
int buffer_index = 0;

File myFile;
int fileCounter;
char fileName[15];
int FILE_COUNTER_ADDRESS = 0;


void processReceivedData(const char* data);

void setup() {
  Serial.begin(115200);  // Initialize Serial communication
  while (!Serial)
    ;

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed. Things to check:");
    Serial.println("1. Is a card inserted?");
    Serial.println("2. Is your wiring correct?");
    Serial.println("3. Did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true)
      ;
  }
  Serial.println("Initialization done.");
  fileCounter = EEPROM.read(FILE_COUNTER_ADDRESS);
    Serial.print("filw xounrw");
  Serial.println(fileCounter);

  
  snprintf(fileName, sizeof(fileName), "data%d.txt", fileCounter);
  Serial.println(fileName);
  fileCounter++;
  delay(100);
 
  EEPROM.write(FILE_COUNTER_ADDRESS, fileCounter);
}

void loop() {
  // Serial.print(Serial.available());

  if (Serial.available()) {  // Wait until 6 bytes are available (header, data, footer)
    char c = Serial.read();
    // Serial.print(c);

    // Check header
    if (c == HEADER) {
      buffer_index = 0;  // Reset buffer index
    } else if (c == FOOTER) {
      // Null-terminate the string and process data
      received_buffer[buffer_index] = '\0';
      processReceivedData(received_buffer);
    } else {
      // Store data into the buffer
      if (buffer_index < sizeof(received_buffer) - 1) {
        received_buffer[buffer_index++] = c;
      }
    }


    // snprintf(dataBuffer, BUFFER_SIZE, "Data: %lu", receivedData);

    // Open the file for appending
  }
}

void processReceivedData(const char* data) {
  // Process the data as needed
 
  myFile = SD.open(fileName, FILE_WRITE);
  // Serial.print("Opening ");

  // Serial.println(myFile);

  // Serial.print("Received Data: ");
  Serial.println(data);
  if (myFile) {
    // Write data to SD card
    myFile.print("Time:, ");
    myFile.print(millis());  // Write the current time in milliseconds
    myFile.print(", ,");
    myFile.println(data);
    myFile.close();  // Close the file
    // Serial.println("Data written to SD card:");
    // Serial.write(data);
  } else {
    Serial.println("Error opening data.txt");
  }
}
