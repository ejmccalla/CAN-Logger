import serial
import time
import sys
import os

#-------------------------------------------------------------------------------
#                       !!! START USER UPDATE !!!
COM_PORT = 'COM3'                                                               # Windows COM port # the Teensy is connected to
BAUD_RATE = 1000000
#                        !!! END USER UPDATE !!!
#-------------------------------------------------------------------------------

FILENAME_SIZE = 23                                                              # CAN-Logger file names are 23 bytes long
FILESIZE_SIZE = 10                                                              # CAN-Logger file is is uint32_t...11 bytes
BLOCK_SIZE = 512                                                                # CAN-Logger logs and sends data in 512 byte blocks
START_CMD = str.encode( "START" ) + b'\x00'                                     # CAN-Logger serial start command

#-------------------------------------------------------------------------------
# Borrowed from https://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console
def PrintProgress (iteration, total, prefix = 'Progress:', suffix = 'Complete', decimals = 1, length = 50, fill = '█', printEnd = "\r"):
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
    if iteration == total: print()

#-------------------------------------------------------------------------------
# This function will go through the steps to receive a log file from the Teensy
def GetFileFromTeensy ():
    rv = True

    fileSize = None
    ser.write( START_CMD )                                                      # Request the file size
    timeout = time.time() + 2                                                   # Wait for a couple seconds to get response back
    while ( True ):
        buf = ser.read( FILESIZE_SIZE )
        if ( buf ):
            #print( len(buf), buf )
            fileSize = int( buf.decode() )
            break
    
        elif ( time.time() > timeout ):
            rv = False
            break
    
    if ( fileSize ):                                                            # Move on if the file size was received
        print ( "File Name: %s, File Size: %s" % ( fileName , fileSize ) )
    else:
        print ( "Didn't receive file size for %s" % (fileName) )
        return rv
    
    byteCnt = 0
    blocks = []
    ser.write( START_CMD )                                                      # Request the file
    timeout = time.time() + 120                                                 # Wait for 2 minutes to get response back
    while ( True ):
        buf = ser.read( BLOCK_SIZE )
        if ( buf ):                                                             # Got some data                 
            if ( len (buf) == 512 ):                                            # and it's a full block as expected
                blocks.append( buf )
                byteCnt += BLOCK_SIZE
                PrintProgress( byteCnt, fileSize )
                if ( byteCnt == fileSize ):
                    with open( fileName, 'wb' ) as fid:
                        for block in blocks: 
                            fid.write( block )
                    break

            else:                                                               # Only a partial block, does the Teensy need to slow down??
                print( "Expected buffer size: 512, received: %s " % ( len ( buf ) ) )
                rv = False
                break

        elif ( time.time() > timeout ):                                         # Timed out before all data was received
                print( "Failed to receive full file: %s / %s" % ( byteCnt, fileSize ) )
                rv = False
                break

    return rv

#-------------------------------------------------------------------------------
#                           !!! MAIN PROGRAM !!!
retries = 5
while (True):                                                                   # Setup the serial connection to the Teensy
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

print ("Connected to the Teensy, requesting CANLogger files...")

while ( True ):                                                                 # File request loop
    fileName = None
    #ser.reset_input_buffer()
    ser.write( START_CMD )                                                      # Request the file name
    timeout = time.time() + 2                                                   # Wait for a couple seconds to get response back
    while ( True ):
        buf = ser.read( FILENAME_SIZE )
        if ( buf ):
            #print( len(buf), buf )
            fileName = buf.decode()
            break
    
        elif ( time.time() > timeout ):
            print ("No more files to transfer...")
            break

    if ( fileName ):                                                            # Begin the file transfer process
        if not GetFileFromTeensy():                                             # and exit if something goes wrong
            break
    

    
     

