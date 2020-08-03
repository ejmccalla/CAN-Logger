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
FILESIZE_SIZE = 10                                                              # CAN-Logger file is is uint32_t...10 bytes
BLOCK_SIZE = 512                                                                # CAN-Logger logs and sends data in 512 byte blocks
START_CMD = str.encode( "START" ) + b'\x00'                                     # CAN-Logger serial start command
SKIP_CMD = str.encode( "SKIP" ) + b'\x00'                                       # CAN-Logger serial skip command

#-------------------------------------------------------------------------------
#  printProgress
#
#    This function will print to the standard output with a progress bar.  This
#    code was copeid from:     
#    stackoverflow.com/questions/3173320/text-progress-bar-in-the-console
#
#   Parameters
#   ----------
#     iteration:  int  The numerator of the status percentage 
#     total       int  The numerator of the status percentage 
#
#   Returns
#   -------
#     none
#-------------------------------------------------------------------------------
def printProgress ( iteration, total ):
  prefix = 'Progress:'
  suffix = 'Complete'
  decimals = 1
  length = 50
  fill = '█'
  printEnd = "\r"
  percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
  filledLength = int(length * iteration // total)
  bar = fill * filledLength + '-' * (length - filledLength)
  print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
  if iteration == total: print()

#-------------------------------------------------------------------------------
#  getFileFromTeensy
#
#    This function will complete the file transer request by getting the file
#    size and then getting the file itself.  The file size is 10 bytes and the
#    file will be a multiple of the CAN-Logger block array size (16-bytes * 
#    kBlockSize), so all components of the data transfer are of known lengths.
#    The calls to read the COM serial ports are all done by trying to read the
#    expected amount of data.  The COM serial read command will only pass if the
#    correct amount of data has been read.  If the read commands aren't getting
#    all of the exected data, then the COM serial read command timeout may need
#    to be increased.
#
#   Parameters
#   ----------
#
#   Returns
#   -------
#     rv  boolean  True = pass, False = fail
#-------------------------------------------------------------------------------
def getFileFromTeensy ():
  rv = True

  fileSize = None
  ser.write( START_CMD )                                                        # Request the file size
  buf = ser.read( FILESIZE_SIZE )
  if ( len( buf ) == FILESIZE_SIZE ):
    fileSize = int( buf.decode() )
    print ( "File Name: %s, File Size: %s" % ( fileName , fileSize ) )

  elif ( len( buf ) > 0 ):
    print( "Expected filesize size: %s, received: %s " %
           ( FILENAME_SIZE, len ( buf ) ) )
    rv = False
    return rv

  else:
    print ( "Didn't receive file size for %s" % (fileName) )
    rv = False
    return rv
    
  byteCnt = 0
  blocks = []
  ser.write( START_CMD )                                                        # Request the file
  while ( True ):
    buf = ser.read( BLOCK_SIZE )
    if ( len( buf ) == BLOCK_SIZE ):
      blocks.append( buf )
      byteCnt += BLOCK_SIZE
      printProgress( byteCnt, fileSize )
      if ( byteCnt == fileSize ):
        with open( fileName, 'wb' ) as fid:
          for block in blocks: 
            fid.write( block )
        break
    else:
      print( "Expected buffer size: 512, received: %s " % 
             ( len ( buf ) ) )
      rv = False
      break

  return rv


#-------------------------------------------------------------------------------
#                           !!! MAIN PROGRAM !!!
#-------------------------------------------------------------------------------
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
        portNum = int( input( "Using %s, verify the COM port number (0-15): " %
                              ( COM_PORT ) ) )
      COM_PORT = 'COM'+str( portNum )
      retries = 5

print ("Connected to the Teensy, requesting CANLogger files...")

while ( True ):                                                                 # File request loop
  fileName = None
  ser.write( START_CMD )                                                        # Request the file name
  buf = ser.read( FILENAME_SIZE )
  if ( len( buf ) == FILENAME_SIZE ):
    fileName = buf.decode()

  elif ( len( buf ) > 0 ):
    print( "Expected filename size: %s, received: %s " %
           ( FILENAME_SIZE, len ( buf ) ) )
    break

  else:                                                                         # No data response, assume no more files
    print ("No more files to transfer...")
    break

  if ( os.path.isfile( fileName ) ):
    print( "File %s already exists, skipping %s " % ( fileName ) )
    ser.write( SKIP_CMD )
  else:
    if not getFileFromTeensy():                                                 # exit if something goes wrong
      break                                                                     # reason should be sent to standard out        
    