#include <SPI.h>
#include "SdFat.h"

SdFatSdioEX sd;                         // The micro SD card object
SdFile root;
SdFile file;

const uint16_t kBufSize = 512;          // This needs to be the block size of the CANLogger.ino
uint16_t bufIdx;
uint8_t buf[ kBufSize ];

const uint8_t cmdBufSz = 6;
size_t bytesRead;
char cmdBuf[ cmdBufSz ];
const char startCmd[] = "START";

const uint8_t fileNameSize= 24;         // Filename format YYYY-MM-DD_HH:MM:SS.bin
char fileName[fileNameSize];

//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  while ( !Serial ) {}

  if ( !sd.begin() ) {
    sd.initErrorHalt();
  }
}
//------------------------------------------------------------------------------
void loop() {

  // Wait for the start comand
  bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
  if ( strcmp( cmdBuf, startCmd ) == 0 ) {
    if (!root.open("/")) {
      sd.errorHalt("open root failed");
    }

    while ( file.openNext( &root, O_RDONLY ) ) {
      if ( !file.isDir() ) {

        // Send the file name without the null character
        file.getName( fileName, (size_t) fileNameSize );
        Serial.write( fileName, (size_t) fileNameSize - 1 );
        
        // Wait to start the next phase
        bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
        while ( strcmp( cmdBuf, startCmd ) != 0 ) {
          delay( 1000 );
          bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
        }

        // Send the file size
        file.printFileSize( &Serial );

        // Wait to start the next phase
        bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
        while ( strcmp( cmdBuf, startCmd ) != 0 ) {
          bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
        }

        // Transfer the file
        bufIdx = 0;
        while ( file.available() ) {
          buf[bufIdx] = file.read();
          bufIdx++;
          if ( bufIdx == kBufSize ) {
            Serial.write( buf, (size_t) kBufSize );
            bufIdx = 0;
            delay (10);
          }
        }
      }
      file.close();
    }
    while (true) {}
  } else {
    delay( 1000 );
  }

}
