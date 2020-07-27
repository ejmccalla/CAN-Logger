import serial
import time
   
BLOCK_SIZE = 512                                            # Teensy will send data in 512 byte blocks

while (True):
    try:
        ser = serial.Serial('COM3', 1000000, timeout=1)     # Setup the serial connection to the Teensy 
        break
    except:
        print ("Attempting to connect to the Teensy...")
        time.sleep(1)

print ("Connected to the Teensy, requesting CANLogger files...")
ser.write( str.encode( "START" ) + b'\x00' )                  # Send a request to the Teensy to begin transfering files

byteCnt = 0
blocks = []
while ( True ):
    buf = ser.read( BLOCK_SIZE )
    if ( buf ):                                             # We've got our block of data from the Teensy
        blocks.append( buf )
        byteCnt += BLOCK_SIZE
        print( byteCnt )
    else:                                                   # The read timed out
        if ( byteCnt == 0 ):                                # Haven't received any data, resend the request for data
            ser.write( str.encode( "START" ) + b'\x00' )
        else:                                               # We can assume that the file has been fully transferred
            with open( "test.bin", 'wb' ) as fid:           # Save the file
                for block in blocks: 
                    fid.write( block )
            break

