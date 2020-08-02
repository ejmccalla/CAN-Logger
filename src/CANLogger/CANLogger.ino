//------------------------------------------------------------------------------
//  FRC CANbus Logger
//
//    The FRC CANbus logger is used to capture all CANbus traffic on an FRC
//    robot.  A FIFO with semaphores is used to decouple micro-SD card writes
//    from the data acquisition.  To work with the shared resource, the ChibiOS
//    real-time OS (RTOS) is used to implement the semaphores.  The CANbus data
//    acquisition is done using the FlexCAN library.  This library grabs the 
//    CANbud data through an interrupt-driven process.  The library also provides
//    a class for the user to provide a callback function.  The callback function
//    is used to load data into the FIFO.  The data is moved from the FIFO to the
//    micro-SD card by using the SdFat library.  Whenver the FIFO is loaded with
//    new block of data, the counting semaphore will signal to the micro-SD write
//    thread that there is data available for writing.
//
//    The logger is started and stopped by commands sent over the CANbus from the
//    RoboRio.  The logger expects the CAN frames will be sent using teh default
//    team and device numbers which are 8 and 10, respectively.  Furthermore, the
//    CANbus logger is designed to be started/stopped only once per power-cycle.
//    In other words, the logger cannot be started/stopped/started/stopped...
//
//  HARDWARE
//
//    The supported hardware is the Teensy 3.6 with a dual-CAN hat.  The products
//    are linked below.
//
//    https://www.pjrc.com/store/teensy36.html
//    https://www.tindie.com/products/Fusion/dual-can-bus-adapter-for-teensy-35-36/
//
//  LIBRARIES
//
//    This project is built using the teensyduino software add-on for the Arduino.
//    Including the standard Arduino and teensyduino installations, the ChibiOS and
//    SdFat libraries also need to be installed.
//
//    Teensyduino
//      https://www.pjrc.com/teensy/teensyduino.html
//
//    ChRt Library for Teensy
//      https://github.com/greiman/ChRt
//
//    SdFat Library for Teensy
//      https://github.com/greiman/SdFat
//
//
//  RESOURCES
//    FRC CAN Spec
//      https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
//
//    FRC Java CAN Class
//      https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/CAN.html
//
//    ChibiOS Counting Semaphores
//      http://chibiforge.org/doc/20.3/full_rm/group__semaphores.html
//
//------------------------------------------------------------------------------

#include "SdFat.h"
#include "ChRt.h"
#include "FlexCAN_T4.h"

// If USE_CAN0 isn't defined, then CAN1 will be assumed and used.  See the link
// above for the dual CAN hardware usage.
//#define USE_CAN0

// Uncomment the define below to have debug messages sent out to the USB COM port
#define DEBUG

#define LED 13

// Bit definitions for failures
#define SD_START_FAILED          ( BIT0 )
#define SD_FILE_OPEN_FAILED      ( BIT1 )
#define SD_FILE_CLOSE_FAILED     ( BIT2 )
#define SD_FILE_WRITE_FAILED     ( BIT3 )
#define CAN_FIFO_OVERRUN         ( BIT4 )
#define CAN_BUFFER_OVERRUN       ( BIT5 )

SdFatSdioEX sd;                                           // The micro SD card object
File file;                                                // File object to write data to the SD card
SdFile root;                                              // SD file object to read the root of the FS
SdFile txferFile;                                         // SD file object of the file to be transferred

#ifdef USE_CAN0                                           // The CAN-bus oject, Teensy 3.6 currently
  FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> CanBus;       // supports 2 buses via the hardware above
#else
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanBus;
#endif

const uint8_t kTeamNumber = 8;                            // CAN frame team number
const uint8_t kDeviceType = 10;                           // CAN frame device number
const uint16_t kStopLogging = 0;                          // CAN frame stop logging command API group/index
const uint16_t kStartLogging = 1;                         // CAN frame start logging command API group/index
const uint16_t kRequestStatus = 16;                       // CAN request data frame command API group/index
const size_t kBlockSize = 32;                             // Block size, the number of CAN messages
const size_t kFifoSize = 32;                              // FIFO size, the number of blocks
const char kStartCmd[] = "START";                         // USB serial incoming START command
const uint8_t kFileNameSize= 24;                          // Filename format YYYY-MM-DD_HH:MM:SS.bin

typedef struct save_CAN_message_t {                       // 16 byte CAN message to save
  uint32_t arbId = 0;
  uint16_t timeStamp = 0;
  struct {
      uint8_t extended:1;
      uint8_t remote:1;
      uint8_t overrun:1;
      uint8_t reserved:5;
  } flags;
  uint8_t length = 8;
  uint8_t buf[8] = { 0 };
} save_CAN_message_t;

typedef struct block_t {                                  // Write data blocks to FIFO/SD rather than individual CAN messages
    save_CAN_message_t frame_array[ kBlockSize ];         
} block_t;
block_t block;
size_t blockHead;
save_CAN_message_t saveCanMessage;

block_t fifo[ kFifoSize ];                                // This FIFO is the go-between of the CAN thread and SD thread
size_t fifoHead = 0;                                      // FIFO head index
size_t fifoTail = 0;                                      // FIFO tail index
uint32_t fifoOverrun = 0;                                 // Count the FIFO overruns
typedef enum {                                            // Logger states, one for each thread
  STOPPED,
  LOGGING
} loggerState_t ;
loggerState_t sdState = STOPPED;
loggerState_t canState = STOPPED;

uint8_t sdFailBitMask = 0;                                // Failing bitmask for the SD thread
uint8_t canFailBitMask = 0;                               // Failing bitmask for the CAN thread

char fileName[] = "2000-00-00_00-00-00.bin";              // Filename format YYYY-MM-DD_HH:MM:SS
char txferFileName[kFileNameSize];                        // Filename of file to transfer

const uint8_t cmdBufSz = 6;                               // USB serial incoming command buffer size
char cmdBuf[ cmdBufSz ];                                  // USB serial incoming command buffer
size_t bytesRead;                                         // USB serial incoming bytes read
uint16_t txferBufIdx;                                     // USB serial outgoing buffer index
uint8_t txferBuf[ 16 * kBlockSize ];                      // USB serial outgoing buffer

SEMAPHORE_DECL ( startLogging, 0 );                       // Used to signal start of logging
SEMAPHORE_DECL ( stopLogging, 0 );                        // Used to signal logging is finished
SEMAPHORE_DECL ( fifoData, 0 );                           // Count of blocks in the FIFO
SEMAPHORE_DECL ( fifoSpace, kFifoSize );                  // Count of free blocks in the FIFO


//------------------------------------------------------------------------------
void printFrame ( const CAN_message_t &frame ) {
  Serial.print("MB "); Serial.print(frame.mb);
  Serial.print("  OVERRUN: "); Serial.print(frame.flags.overrun);
  Serial.print("  LEN: "); Serial.print(frame.len);
  Serial.print(" EXT: "); Serial.print(frame.flags.extended);
  Serial.print(" RTR: "); Serial.print(frame.flags.remote);
  Serial.print(" TS: "); Serial.print(frame.timestamp);
  Serial.print(" BUS: "); Serial.print(frame.bus);
  Serial.print(" ID: "); Serial.print(frame.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < frame.len; i++ ) {
    Serial.print(frame.buf[i], HEX); Serial.print(" ");
  } Serial.println();

}


//------------------------------------------------------------------------------
void processFrame ( const CAN_message_t &frame ) {
  switch ( canState ) {

    case STOPPED:
      if ( ( ( ( frame.id >> 16 ) & 0xFF ) == kTeamNumber ) &             // Wait for the start logging command
           ( ( ( frame.id >> 24 ) & 0x1F ) == kDeviceType ) &
           ( ( ( frame.id >> 6 ) & 0x3FF ) == kStartLogging ) ) {
        fileName[0]  = ( ( frame.buf[0] + 2000 ) / 1000 ) % 10 + '0';
        fileName[1]  = ( ( frame.buf[0] + 2000 ) / 100 ) % 10 + '0';
        fileName[2]  = ( ( frame.buf[0] + 2000 ) / 10 ) % 10 + '0';
        fileName[3]  = ( frame.buf[0] + 2000 ) % 10 + '0';
        fileName[5]  = ( frame.buf[1] / 10 ) % 10 + '0';
        fileName[6]  = frame.buf[1] % 10 + '0';
        fileName[8]  = ( frame.buf[2] / 10 ) % 10 + '0';
        fileName[9]  = frame.buf[2] % 10 + '0';
        fileName[11] = ( frame.buf[3] / 10 ) % 10 + '0';
        fileName[12] = frame.buf[3] % 10 + '0';
        fileName[14] = ( frame.buf[4] / 10 ) % 10 + '0';
        fileName[15] = frame.buf[4] % 10 + '0';
        fileName[17] = ( frame.buf[5] / 10 ) % 10 + '0';
        fileName[18] = frame.buf[5] % 10 + '0';
        #ifdef DEBUG
          printFrame( frame );
        #endif
        chSemSignal ( &startLogging );                                    // Signal to the SD thread to start logging,
        canState = LOGGING;                                               // update the CAN state to logging,
        blockHead = 0;                                                    // and initialize the block index / FIFO
        fifoOverrun = 0;                                                  // overrun count to 0
      }      
      break;
  
    case LOGGING:
      if ( ( ( ( frame.id >> 16 ) & 0xFF ) == kTeamNumber ) &             // Check for the stop logging command
           ( ( ( frame.id >> 24 ) & 0x1F ) == kDeviceType ) &             // The data in the partially filled block
           ( ( ( frame.id >> 6 ) & 0x03FF ) == kStopLogging ) ) {         // will be lost
        #ifdef DEBUG
          printFrame( frame );
        #endif
        chSemSignal ( &stopLogging );                                     // Signal to the SD thread to stop logging and
        canState = STOPPED;                                               // update the CAN state to stopped
      
      } else {
        saveCanMessage.arbId = frame.id;                                  // Copy the CAN frame contents we wish to save
        saveCanMessage.timeStamp = frame.timestamp;
        saveCanMessage.flags.extended = frame.flags.extended;
        saveCanMessage.flags.overrun = frame.flags.overrun;
        if ( saveCanMessage.flags.overrun == 1 ) {
          canFailBitMask |= CAN_BUFFER_OVERRUN;
          #ifdef DEBUG
            Serial.println( F( "CAN: Buffer Overrun" ) );
          #endif
        }
        saveCanMessage.flags.remote = frame.flags.remote;
        saveCanMessage.length = frame.len;
        for (uint8_t i = 0; i < 8; i++) {
          saveCanMessage.buf[i] = frame.buf[i];
        }
        block.frame_array[blockHead] = saveCanMessage;                    // Save the frame to the block array
        if ( blockHead < ( kBlockSize - 1 ) ) {                           // Increment the block index
          blockHead = blockHead + 1;
        
        } else if ( chSemWaitTimeout( &fifoSpace, TIME_IMMEDIATE ) != MSG_OK ) { // There is no space in the FIFO and we have an overrun condition
          fifoOverrun++;
          canFailBitMask |= CAN_FIFO_OVERRUN;
          #ifdef DEBUG
            Serial.println( F( "CAN: FIFO Overrun" ) );
          #endif
        
        } else {
          fifo[fifoHead] = block;                                         // Copy the block into the FIFO,
          chSemSignal( &fifoData );                                       // signal to the SD thread there is data to write,
          fifoHead = fifoHead < ( kFifoSize - 1 ) ? fifoHead + 1 : 0;     // update the FIFO head index, and
          blockHead = 0;                                                  // re-initilize the block index
        }
      }
      break;
  }
  if ( ( ( ( frame.id >> 16 ) & 0xFF ) == kTeamNumber ) &                 // Check for remote data request
       ( ( ( frame.id >> 24 ) & 0x1F ) == kDeviceType ) &
       ( ( ( frame.id >> 6 ) & 0x3FF ) == kRequestStatus ) &
       ( frame.flags.remote == 1 ) ) {
    CAN_message_t response;
    response.id = ( kDeviceType << 24 ) + ( kTeamNumber << 16 ) + ( 16 << 6 );
    response.len = 1;
    response.buf[0] = 33;
    response.flags.extended = 1;
    Serial.println( F( "CAN: Remote response" ) );
    printFrame( frame );
    printFrame( response );
    CanBus.write( response );
  }

}


//------------------------------------------------------------------------------
THD_WORKING_AREA( WACT, 200 );                                            // Declare a stack with 200 bytes beyond context switch and interrupt needs
THD_FUNCTION( CT, arg ) {                                                 // Declare the CAN thread function, which simply calls
  while (TRUE) {                                                          // the CAN events method which will call our processFrame
    CanBus.events();                                                      // frame function if there is a frame to process
  }
}

//------------------------------------------------------------------------------
void FlashErrorCode ( uint8_t ec ) {
  digitalWrite( LED, LOW );
  while ( true ) {
    delay(4000);
    for (uint8_t i = 0; i < 8; i++) {
      if ( ( ec >> i) & 0x1 ) {
        digitalWrite( LED, HIGH );
        delay(2000);                // Long for 1
        digitalWrite( LED, LOW );
        delay(1000);
      } else {
        digitalWrite( LED, HIGH );
        delay(1000);                // Short for 0
        digitalWrite( LED, LOW );
        delay(1000);
      }
    }
  }   
}

//------------------------------------------------------------------------------
void TransferLogs ( void ) {
  if (!root.open("/")) {
    sd.errorHalt("open root failed");
  }

  uint8_t alreadyHaveFileName = 1;
  uint8_t ledState = HIGH;
  uint8_t blockTxfrCnt = 0;
  while ( txferFile.openNext( &root, O_RDONLY ) ) {
    if ( !txferFile.isDir() ) {

      // Send the name without the null character
      if ( !alreadyHaveFileName ) {
        bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
        while ( strcmp( cmdBuf, kStartCmd ) != 0 ) {
          delay( 100 );
          bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
        }
        alreadyHaveFileName = 0;       
      }
      txferFile.getName( txferFileName, (size_t) kFileNameSize );
      Serial.write( txferFileName, (size_t) kFileNameSize - 1 );

      // Send the size when requested
      bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
      while ( strcmp( cmdBuf, kStartCmd ) != 0 ) {
        delay( 100 );
        bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
      }        
      txferFile.printFileSize( &Serial );

      // Send the file when requested
      bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
      while ( strcmp( cmdBuf, kStartCmd ) != 0 ) {
        delay( 100 );
        bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
      }
      txferBufIdx = 0;
      while ( txferFile.available() ) {
        txferBuf[ txferBufIdx ] = txferFile.read();
        txferBufIdx++;
        if ( txferBufIdx == ( 16 * kBlockSize ) ) {
          delay (10);
          Serial.write( txferBuf, (size_t) ( 16 * kBlockSize ) );
          txferBufIdx = 0;
          blockTxfrCnt++;
          if ( ( ledState == LOW ) & ( blockTxfrCnt == 2 ) ) {
            digitalWrite( LED, HIGH );
            ledState = HIGH;
            blockTxfrCnt = 0;
          } else if ( ( ledState == HIGH ) & ( blockTxfrCnt == 2 ) ) {
            digitalWrite( LED, LOW );
            ledState = LOW;
            blockTxfrCnt = 0;
          }
        }
      }
      digitalWrite( LED, HIGH );
      ledState = HIGH;
    }
    txferFile.close();
  }
  root.close();
  delay( 4000 );  // The PC-side will timeout after 2 seconds and stop sending requests for files
  Serial.clear();
}

//------------------------------------------------------------------------------
void chStartup() {
  chThdCreateStatic( WACT, sizeof(WACT), NORMALPRIO, CT, NULL );          // Startup the CAN thread
}


//------------------------------------------------------------------------------
void setup ( void ) {
  pinMode( LED, OUTPUT );                                                 // Setup the LED output
  
  Serial.begin( 9600 );
  //while ( !Serial ) {}                                                    // Wait for USB Serial COMs
  
  if ( !sd.begin() ) {                                                    // Startup the SD interface
    #ifdef DEBUG
      Serial.println( F( "Setup: Failed to initialize SD" ) );
    #endif    
    sdFailBitMask |= SD_START_FAILED;
    FlashErrorCode( sdFailBitMask );                                      // Does not return
  }
  #ifdef DEBUG
    Serial.println( F( "Setup: Initialized SD Card Interface" ) );
  #endif

  CanBus.begin();                                                         // Startup the CAN interface
  CanBus.setBaudRate(1000000);
  CanBus.setMaxMB(16);
  CanBus.enableFIFO();
  CanBus.enableFIFOInterrupt();
  CanBus.onReceive( processFrame );
  #ifdef DEBUG
    Serial.println( F( "Setup: Initialized CAN Interface" ) );
  #endif

  digitalWrite( LED, HIGH );

  chBegin( chStartup );                                                   // Start kernel - loop() becomes main thread
  while (true) {}                                                         // chBegin() resets stacks and should never return
}


//------------------------------------------------------------------------------
void loop () {

  switch ( sdState ) {

    case STOPPED:
      
      #ifdef DEBUG
        Serial.println( F( "Loop: Waiting for start command" ) );
      #endif
      while ( chSemWaitTimeout( &startLogging, TIME_IMMEDIATE ) != MSG_OK ) { // Wait until a start logging command is received
        bytesRead = Serial.readBytes( cmdBuf, (size_t) cmdBufSz );
        if ( ( (uint8_t) bytesRead == cmdBufSz ) & ( strcmp( cmdBuf, kStartCmd ) == 0 ) ) {  // Looks like the Teensy is connected to a PC looking to download files
          TransferLogs();
        }
      }
      #ifdef DEBUG
        Serial.println( F( "Loop: Received for start command" ) );
        Serial.println( F( fileName ) );
      #endif
      if ( !file.open( fileName, O_CREAT | O_WRITE | O_TRUNC ) ) {            // The start command sets the filename
        sdFailBitMask |= SD_FILE_OPEN_FAILED;
        #ifdef DEBUG
          Serial.println( F( "Loop: SD file open failed" ) );
        #endif
        FlashErrorCode( sdFailBitMask );                                      // Does not return
      }
      sdState = LOGGING;
      break;
  
    case LOGGING:

      while ( chSemWaitTimeout( &stopLogging, TIME_IMMEDIATE ) != MSG_OK ) {    // Wait until a stop logging command is received
        if ( chSemWaitTimeout( &fifoData, TIME_IMMEDIATE ) == MSG_OK ) {        // Check if there's data ready to be written to SD
          block_t *block = &fifo[fifoTail];
          if ( kBlockSize * 16 != file.write( block, kBlockSize * 16 ) ) {
              sdFailBitMask |= SD_FILE_WRITE_FAILED;
              #ifdef DEBUG
                Serial.println( F( "Loop: SD write failed" ) );
              #endif               
              FlashErrorCode( sdFailBitMask );                                      // Does not return
          }
          chSemSignal( &fifoSpace );                                            // Release the FIFO block
          fifoTail = fifoTail < ( kFifoSize - 1 ) ? fifoTail + 1 : 0;           // Advance FIFO tail index
        }
      }
      #ifdef DEBUG
        Serial.println( F( "Loop: Received stop logging command" ) );
      #endif    
      if ( !file.close() ) {
        sdFailBitMask |= SD_FILE_CLOSE_FAILED;
        #ifdef DEBUG
          Serial.println( F( "Loop: SD file close failed" ) );
        #endif
        FlashErrorCode( sdFailBitMask );                                      // Does not return 
      }
      sdState = STOPPED;
      break;

  }

}
