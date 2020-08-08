import os
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from DecodeTools import BitTracker, hex2Bin, twosComp, flipBits

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
    ------
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
    ------
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
    

def isCTRETalonStatus1 ( apiClass, apiIndex, ):
  rv = False
  if ( apiClass == 0 and apiIndex == 2 ): rv = True
  return rv

def isCTREVictorStatus1 ( apiClass, apiIndex, ):
  rv = False
  if ( apiClass == 0 and apiIndex == 2 ): rv = True
  return rv

def decodeCTRETalonStatus1 ( data ):
  rv = {}
  # Percent output:
  #       Byte 0  Byte 1
  # Bits  7:0     15:13
  binVal = hex2Bin( data[ 0:2 ] ) + hex2Bin( data[ 2:4 ] )[ 0:3 ]
 
  # Bit 13 = bit 19 XOR (bit13 XOR bit 16)
  bit13 = int(hex2Bin( data[ 4:6 ] )[ 5 ], 2) ^ (int(binVal[10],2) ^ int(hex2Bin( data[ 4:6 ] )[ 7 ], 2))

  # Bit 14 = bit 19 XOR (NOT bit14 XOR Bit 17)
  bit14 = int(hex2Bin( data[ 4:6 ] )[ 5 ], 2) ^ (1 - (int(binVal[9],2) ^ int(hex2Bin( data[ 4:6 ] )[ 6 ], 2)))

  # Bit 15 = bit 19 XOR (NOT bit15 XOR Bit 18)
  bit15 = int(hex2Bin( data[ 4:6 ] )[ 5 ], 2) ^ (1 - (int(binVal[8],2) ^ int(hex2Bin( data[ 4:6 ] )[ 5 ], 2)))

  
  
  # Bits 0 is inverted
  binVal = binVal[ 0:7 ] + flipBits( binVal[ 7 ] ) + str( bit15 ) + str( bit14 ) + str( bit13 )

  # Bits 2, 3, 4, 6 XORed with bit 19


  rv['Output (%)'] = twosComp( int( binVal, 2 ), 11 )
  # Scale to +-100.0
  rv['Output (%)'] *= ( 100.0 / 1023.0 )

  # All frames output all individual bits
  for byte in range( 8 ):
    for bit in reversed( range( 8 ) ):
      rv[ 'Bit %s' % ( 8 * byte + bit ) ] = ( int( data[ (2 * byte):((2 * byte)+2) ], 16 ) & ( 1 << bit ) ) >> bit

  rv['Bit 0']  = 1 - rv['Bit 0']
  rv['Bit 13'] = bit13
  rv['Bit 14'] = bit14
  rv['Bit 15'] = bit15

  return rv

def decodeCTREVictorStatus1 ( data ):
  return decodeCTRETalonStatus1( data )  


#-------------------------------------------------------------------------------
def DecodeFrame( fileName, devType, man, apiClass, apiIndex, devId, timeStamp, flags, length, data ):
  rv = {}
  if ( man == 4 ):
    if ( devType == 2 ):
      if ( isCTRETalonStatus1( apiClass, apiIndex ) ):                          # CTRE:Victor:Status1
        rv = decodeCTRETalonStatus1( data )
    elif ( devType == 1 ):
      if ( isCTREVictorStatus1( apiClass, apiIndex ) ):                         # CTRE:Talon:Status1
        rv = decodeCTREVictorStatus1( data )  
  return rv


#-------------------------------------------------------------------------------
def DecodeCANLogfile ( fileName ):
    """

    Parameters
    ----------
    fileName:
      Full string path and filename

    Return
    ------
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
            row['abrID'] = arbID
            row['deviceType'] = devType
            row['manufacturer'] = man
            row['apiClass'] = apiClass
            row['apiIndex'] = apiIndex
            row['deviceId'] = devId
            row['flags'] = flags
            row['timestamp'] = ( intTimeStamp + timeStampAdder ) / 1e3
            row['length'] = int( length,  16 )
            row['data'] = data

            # Check to see if we know how to decode the data for this frame
            row.update( DecodeFrame( fileName, devType, man, apiClass, apiIndex, devId, timeStamp, flags, length, data ) )

            rows.append( row )


            # Get the next frame
            arbID, timeStamp, flags, length, data = ReadCANFrame( inFile )
    
    return rows


#-------------------------------------------------------------------------------
#                           !!! MAIN PROGRAM !!!
#-------------------------------------------------------------------------------
lfPath = os.path.join( os.path.dirname( os.getcwd() ), 'CANTransfer' )          # The CANDecoder and CANTransfer folders need share same parent folder
rawLFs = [ f for f in os.listdir( lfPath ) if f.endswith( '.bin' ) ]            # Get a list of raw log files
parsedLFs = [ f for f in os.listdir( lfPath ) if f.endswith( '.csv' ) ]         # Get a list of decoded log files

for rawLF in rawLFs:  
  parsedLF = rawLF.split( '.' )[0]+'.csv'
  if parsedLF not in parsedLFs:
    df = pd.DataFrame( DecodeCANLogfile( os.path.join( lfPath, rawLF ) ) )
    #df.to_csv( os.path.join( lfPath, parsedLF ), index=False )
    
    #---------------------------------------------------------------------------
    # The following will output CSV's for each Talon API Class/Index
    talons = df.loc[ ( df[ 'manufacturer' ] == 4 ) &                            # Select the Talons
                     ( df[ 'deviceType' ] == 2 ) ]
    talonDevIds = talons['deviceId'].unique()
    for devId in talonDevIds:
      talonDevId = talons.loc[ talons[ 'deviceId' ] == devId ]                  # Select the Talon for the given ID
      apiClasses = talonDevId['apiClass'].unique()
      for apiClass in apiClasses:
        talonAC = talonDevId.loc[ talonDevId[ 'apiClass' ] == apiClass ]        # Select the API class
        apiIndeces = talonAC['apiIndex'].unique()
        for apiIndex in apiIndeces:
          talonAI = talonAC.loc[ talonAC[ 'apiIndex' ] == apiIndex ]            # Select the API index
          talonAI.to_csv( os.path.join( lfPath, 'talon_%s_api_%s_%s.csv' %      # Ouput to CSV (using append mode)
                          ( devId, apiClass, apiIndex ) ), mode='a', index=False )

    #---------------------------------------------------------------------------
    # The following will output CSV's for each Victor API Class/Index
    # victors = df.loc[ ( df[ 'manufacturer' ] == 4 ) &                            # Select the Victors
    #                   ( df[ 'deviceType' ] == 1 ) ]
    # victorDevIds = victors['deviceId'].unique()
    # for devId in victorDevIds:
    #   victorDevId = victors.loc[ victors[ 'deviceId' ] == devId ]                  # Select the Talon for the given ID
    #   apiClasses = victorDevId['apiClass'].unique()
    #   for apiClass in apiClasses:
    #     victorAC = victorDevId.loc[ victorDevId[ 'apiClass' ] == apiClass ]        # Select the API class
    #     apiIndeces = victorAC['apiIndex'].unique()
    #     for apiIndex in apiIndeces:
    #       victorAI = victorAC.loc[ victorAC[ 'apiIndex' ] == apiIndex ]            # Select the API index
    #       victorAI.to_csv( os.path.join( lfPath, 'victor_%s_api_%s_%s.csv' %      # Ouput to CSV (using append mode)
    #                        ( devId, apiClass, apiIndex ) ), mode='a', index=False )

    #---------------------------------------------------------------------------
    # The following will output a CSV for all Talon motor % outputs
    # talonStatus1 = df.loc[ ( df[ 'manufacturer' ] == 4 ) &                      # Select Talons Status 1
    #                        ( df[ 'deviceType' ] == 2 ) &
    #                        ( df[ 'apiClass' ] == 0 ) &
    #                        ( df[ 'apiIndex' ] == 2 ) ]
    # talonStatus1.to_csv( os.path.join( lfPath, 'talon_motor_output.csv' ),
    #                      columns=[ 'deviceId', 'Output (%)' ],
    #                      mode='a', index=False )

    #---------------------------------------------------------------------------
    # The following will plot all Talon motor % outputs
    # talonStatus1 = df.loc[ ( df[ 'manufacturer' ] == 4 ) &                      # Select Talons Status 1
    #                        ( df[ 'deviceType' ] == 2 ) &
    #                        ( df[ 'apiClass' ] == 0 ) &
    #                        ( df[ 'apiIndex' ] == 2 ) ]
    
    # sns.lineplot( y=talonStatus1['Output (%)'], x=talonStatus1['timestamp'],  style = talonStatus1["deviceId"] )
    # plt.show()

      # talonDevId.to_csv( os.path.join( lfPath, 'talon_%s.csv' % ( devId ) ),
      #                    mode = 'a', index = False )
  
    # victors = df.loc[ ( df['manufacturer'] == 4 ) & ( df['deviceType'] == 1 ) ]
    # talons.to_csv( os.path.join( lfPath, 'victors.csv' ), mode = 'a', index = False )

