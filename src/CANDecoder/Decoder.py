import os
from DecodeTools import BitTracker
import pandas as pd

#-------------------------------------------------------------------------------
def ReadCANFrame ( f ):
    """
    This function will read a single CAN frame saved by the the CAN-Logger.
    Each call to this function will advance the file pointer by 16-bytes.  The
    CAN-Logger stores the following CAN data:

    arbitration ID (4-bytes):   This is a uint32, need to account for endianness
    timestamp (2-bytes):        This is a uint16, need to account for endianness
    flags (1-byte):             
    length (1-byte):
    data (8-bytes):             This is composed of uint8's, no endianness

    Parameters
    ----------
    f : file object
        This the file object returned when opening the CAN-Logger file for input

    Return
    ======
    arbID:
      4-byte hex value
    timeStamp:
      2-byte hex value
    flags:
      1-byte hex value
    length:
      1-byte hex value
    data:
      8-byte hex value
    """
    arbID = f.read(4).hex()
    arbID = arbID[6:8] + arbID[4:6] + arbID[2:4] + arbID[0:2]
    timeStamp = f.read(2).hex()
    timeStamp = timeStamp[2:4] + timeStamp[0:2]
    flags = f.read(1).hex()
    length = f.read(1).hex()
    data = f.read(8).hex()
    return [arbID, timeStamp, flags, length, data]


#-------------------------------------------------------------------------------
def DecodeCANArbID ( arbID ):
    """
    This function will decode the arbitration ID based on the FRC CAN spec:
    https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html

    Parameters
    ----------
    arbID:
      A 32-bit hex value without the leading '0x'

    Return
    ======
    deviceType:
      5-bit integer
    manufacturer:
      8-bit integer
    apiClass:
      6-bit integer
    apiIndex:
      4-bit integer
    deviceId:
      6-bit integer
    """    
    binArbId = bin( int( arbID, 16 ) )[2:].zfill( 32 )[::-1]
    deviceType = int( binArbId[24:29][::-1], 2 )
    manufacturer = int( binArbId[16:24][::-1], 2 )
    apiClass = int( binArbId[10:16][::-1], 2 )
    apiIndex = int( binArbId[6:10][::-1], 2 )
    deviceId = int( binArbId[0:6][::-1], 2 )
    return [deviceType, manufacturer, apiClass, apiIndex, deviceId]
    

#-------------------------------------------------------------------------------
def DecodeCANLogfile ( fileName ):
    """

    Parameters
    ----------
    fileName:
      Full string path and filename

    Return
    ======
      rows:
        A list of dictionaries containing the decoded CAN frames
    
    """
    rows = []
    with open( fileName, 'rb' ) as inFile:
        lastTimeStamp = -1.0
        timeStampAdder = 0
        
        arbID, timeStamp, flags, length, data = ReadCANFrame( inFile )
        while ( arbID ):
            devType, man, apiClass, apiIndex, devId = DecodeCANArbID( arbID )
            
            # The 2-byte timestamp will rollover every 2^16, track the
            # rollovers and add them to the local timestamp
            intTimeStamp = int( timeStamp,  16 )
            if intTimeStamp < lastTimeStamp:
                timeStampAdder += 1 << 16
            lastTimeStamp = intTimeStamp
            
            # Gather the decoded data into a dictionary and add it to our list
            row = {}
            row['deviceType'] = devType
            row['manufacturer'] = man
            row['apiClass'] = apiClass
            row['apiIndex'] = apiIndex
            row['deviceId'] = devId
            row['flags'] = flags
            row['timestamp'] = ( intTimeStamp + timeStampAdder ) / 1e3
            row['length'] = int( length,  16 )
            row['data'] = data
            rows.append( row )

            # Get the next frame
            arbID, timeStamp, flags, length, data = ReadCANFrame( inFile )
    
    return rows



#-------------------------------------------------------------------------------
df = pd.DataFrame( DecodeCANLogfile( '.bin' ) )
df.to_csv( 'Output.csv', index=False )
