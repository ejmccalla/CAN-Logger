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

  bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
  if ( strcmp( cmdBuf, startCmd ) == 0 ) {  // Serial COM is asking to transfer file
    if (!root.open("/")) {
      sd.errorHalt("open root failed");
    }

    while ( file.openNext( &root, O_RDONLY ) ) {
      if ( !file.isDir() ) {
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

  } else {
    delay( 1000 );
  }

}
