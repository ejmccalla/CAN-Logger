import os
from DecodeTools import DecodeTools
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
    A list of 5 CAN frame elements

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
def DecodeCANArbID (arbID):
    """
    
    Parameters
    ----------

    Return
    ======
    
    """    
    binArbId = bin( int( arbID, 16 ) )[2:].zfill( 32 )[::-1]
    deviceType = int( binArbId[24:29][::-1], 2 )
    manufacturer = int( binArbId[16:24][::-1], 2 )
    apiClass = int( binArbId[10:16][::-1], 2 )
    apiIndex = int( binArbId[6:10][::-1], 2 )
    deviceId = int( binArbId[0:6][::-1], 2 )
    return [deviceType, manufacturer, apiClass, apiIndex, deviceId]
    
#-------------------------------------------------------------------------------
def DecodeCANLogfile ():
    """
    
    Parameters
    ----------

    Return
    ======
    
    """  
    with open( '2020-07-23_17-36-54.bin', 'rb' ) as inFile:
        Status1BitMonitor = None
        lastTimeStamp = -1.0
        timeStampAdder = 0

        arbID, timeStamp, flags, length, data = ReadCANFrame( inFile )
        while ( arbID ):
            deviceType, manufacturer, apiClass, apiIndex, deviceId = DecodeCANArbID( arbID )
            intTimeStamp = int( timeStamp,  16 )
            if intTimeStamp < lastTimeStamp:
                timeStampAdder += 1 << 16
            lastTimeStamp = intTimeStamp
            
            if ( deviceType == 2 and apiClass == 0 and deviceId == 0 and apiIndex == 2):
                if not Status1BitMonitor:
                    Status1BitMonitor = DecodeTools( data )
                Status1BitMonitor.UpdateBitChanges( data )
                
                binary_string = bin( int( data[0:2], 16 ) )[2:].zfill( 8 ) + bin( int( data[2:4], 16 ) )[2:].zfill( 8 )[0:3]
                print( data, Status1BitMonitor.TwosComp( int( binary_string, 2 ), len (binary_string ) ) / 1023.0, Status1BitMonitor.GetPriorChangingBits()  )
                
                row = {}
                #row['deviceType'] = deviceType
                #row['manufacturer'] = manufacturer
                #row['apiClass'] = apiClass
                #row['apiIndex'] = apiIndex
                #row['deviceId'] = deviceId
                #row['flags'] = flags
                row['timestamp'] = ( intTimeStamp + timeStampAdder ) / 1e3
                #row['length'] = int( length,  16 )
                row['data'] = data
                row['motor_output'] = Status1BitMonitor.TwosComp( int( binary_string, 2 ), len (binary_string ) ) / 1023.0
                row['changing_bits'] = Status1BitMonitor.GetPriorChangingBits() 
                
                
                
                rows.append( row )


            arbID, timeStamp, flags, length, data = ReadCANFrame( inFile )



#----------------------------------------------------------------------------------------------------------------------
rows = []
DecodeCANLogfile()
df = pd.DataFrame(rows)
df.to_csv( 'Output.csv', index=False )
