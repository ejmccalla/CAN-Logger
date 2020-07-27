import serial
import time
import sys
import os

FILENAME_SIZE = 24                          # CAN-Logger file names are 24 bytes long
FILESIZE_SIZE = 11                          # CAN-Logger file is is uint32_t...11 bytes
BLOCK_SIZE = 512                            # CAN-Logger logs and sends data in 512 byte blocks
START_CMD = str.encode( "START" ) + b'\x00' # CAN-Logger serial start command
COM_PORT = 'COM3'                           # This is PC-specific
BAUD_RATE = 1000000                         # Also PC-specific

#-------------------------------------------------------------------------------
# Borrowed from https://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console
def PrintProgress (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()


#-------------------------------------------------------------------------------
# Setup the serial connection to the Teensy
retries = 5
while (True):
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)     
        break
    except:
        print ("Attempting to connect to the Teensy...")
        time.sleep(1)
        retries-=1
        if ( retries == 0 ):
            portNum = -1
            while ( portNum < 0 or portNum > 15 ):
                portNum = int( input( "Using %s, verify the COM port number (0-15): " % ( COM_PORT ) ) )
            COM_PORT = 'COM'+str( portNum )
            retries = 5

#-------------------------------------------------------------------------------
# Send a request to the Teensy to begin transfering files.
print ("Connected to the Teensy, requesting CANLogger files...")
ser.write( START_CMD ) 

#-------------------------------------------------------------------------------
# Get the file name and star the next phase
while ( True ):
    buf = ser.read( FILENAME_SIZE )
    if ( buf ):                                             # We've got our file name
        fileName = buf.decode()
        break

ser.write( START_CMD )

#-------------------------------------------------------------------------------
# Get the file size and star the next phase
while ( True ):
    buf = ser.read( FILESIZE_SIZE )
    if ( buf ):                                             # We've got our file name
        fileSize = int( buf.decode() )
        break

ser.write( START_CMD )

#-------------------------------------------------------------------------------
# Get the file
print ( "File Name: %s, File Size: %s" % ( os.path.join( os.getcwd(), fileName ), fileSize ) )
byteCnt = 0
blocks = []
while ( True ):
    buf = ser.read( BLOCK_SIZE )
    if ( buf ):                                             # We've got our block of data
        if ( len (buf) == 512 ):
            blocks.append( buf )
            byteCnt += BLOCK_SIZE
            PrintProgress(byteCnt, fileSize, prefix = 'Progress:', suffix = 'Complete', length = 50)

        else:
            print( "Buffer size != 512: %s " % ( len ( buf ) ) )
    else:                                                   # The read timed out
        if ( byteCnt == 0 ):                                # Haven't received any data, resend the request for data??
            print( "Didn't receive any file data" )
            break
            #ser.write( START_CMD )
        elif ( byteCnt == fileSize  ) :                     # The full file has been received
            with open( fileName, 'wb' ) as fid:
                for block in blocks: 
                    fid.write( block )
            break
        else:                                               # Didn't get the full file
            print( "Failed to receive full file: %s / %s" % ( byteCnt, fileSize ) )
            break

