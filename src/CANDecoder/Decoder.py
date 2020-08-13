import os
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from DecodeTools import BitTracker, hex2Bin, twosComp, flipBits, flipBit

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
  b16 = int( hex2Bin( data[ 4:6 ] )[ 7 ], 2)
  b17 = int( hex2Bin( data[ 4:6 ] )[ 6 ], 2)
  b18 = int( hex2Bin( data[ 4:6 ] )[ 5 ], 2)
  b19 = int( hex2Bin( data[ 4:6 ] )[ 4 ], 2)
  b11 = int( hex2Bin( data[ 2:4 ] )[ 4 ], 2)
  b12 = int( hex2Bin( data[ 2:4 ] )[ 3 ], 2)
  b13 = int( binVal[ 10 ], 2 )
  b14 = int( binVal[ 9 ], 2 )
  b15 = int( binVal[ 8 ], 2 )
  b0 = int( binVal[ 7 ], 2 )
  b1 = int( binVal[ 6 ], 2 )
  b2 = int( binVal[ 5 ], 2 )
  b3 = int( binVal[ 4 ], 2 )
  b4 = int( binVal[ 3 ], 2 )
  b5 = int( binVal[ 2 ], 2 )
  b6 = int( binVal[ 1 ], 2 )
  b7 = int( binVal[ 0 ], 2 )

  # Bit 13
  b13 = b13 ^ ( ( b12            & flipBit( b11 ) & b17            & b16            ) | 
                ( b12            & flipBit( b11 ) & flipBit( b17 ) & b16            ) | 
                ( b12            & b11            & b17            & flipBit( b16 ) ) | 
                ( flipBit( b12 ) & flipBit( b11 ) & flipBit( b17 ) & flipBit( b16 ) ) |
                ( flipBit( b12 ) & b11            & b17            & flipBit( b16 ) ) ) 

  # Bit 14
  b14 = b14 ^ ( ( flipBit( b12 ) & flipBit( b11 ) & b18            & b17            ) | 
                ( flipBit( b12 ) & b11            & b18            & flipBit( b17 ) ) | 
                ( b12            & flipBit( b11 ) & flipBit( b18 ) & flipBit( b17 ) ) | 
                ( b12            & b11            & b18            & b17            ) |
                ( flipBit( b12 ) & b11            & flipBit( b18 ) & flipBit( b17 ) ) ) 

  # Bit 15
  b15 = b15 ^ ( ( b12            & flipBit( b11 ) & flipBit( b19 ) & flipBit( b18 ) ) |
                ( flipBit( b12 ) & b11            & flipBit( b19 ) & flipBit( b18 ) ) |
                ( flipBit( b12 ) & flipBit( b11 ) & flipBit( b19 ) & b18          ) )

  # Bit 0 
  b0 = b0 ^ ( ( b12            & flipBit( b11 ) & flipBit( b19 ) & flipBit( b16 ) ) |
              ( b12            & flipBit( b11 ) & flipBit( b19 ) & b16            ) |
              ( b12            & b11            & flipBit( b19 ) & flipBit( b16 ) ) |
              ( flipBit( b12 ) & flipBit( b11 ) & b19            & b16            ) |
              ( flipBit( b12 ) & b11            & flipBit( b19 ) & b16            ) |
              ( flipBit( b12 ) & flipBit( b11 ) & b19            & flipBit( b16 ) ) |
              ( flipBit( b12 ) & b11            & flipBit( b19 ) & flipBit( b16 ) ) )

  # Bit 1 
  b1 = b1 ^ flipBit( b12 & flipBit( b11 ) )
  
  # Bit 2 = bit 2 XOR bit 11
  b2 =  b2 ^ b11

  # Bit 3 = not XOR bit 3 and bit 12
  b3 = flipBit( b3 ^ b12 )

  # Bit 4 = bit 4 XOR not (bit 12 XOR bit 11)
  b4 =  b4 ^ flipBit( b11 ^ b12 )

  # Bit 5 = XOR bit 5 and bit 11
  b5 =  b5 ^ b11

  # Bit 6 = bit 6 XOR (not bit 12 AND not bit 11)
  b6 = b6 ^ ( flipBit( b12 ) & flipBit( b11 ) )

  updatedBinVal = str( b7 ) + str( b6 ) + str( b5 ) + str( b4 ) + str( b3 ) + str( b2 ) + str( b1 ) + str( b0 ) + str( b15 ) + str( b14 ) + str( b13 )

  rv['Output (%)'] = twosComp( int( updatedBinVal, 2 ), 11 )
  # Scale to +-100.0
  rv['Output (%)'] *= ( 100.0 / 1023.0 )
  rv['raw input'] = int( binVal, 2 )

  # All frames output all individual bits
  for byte in range( 8 ):
    for bit in reversed( range( 8 ) ):
      rv[ 'Bit %s' % ( 8 * byte + bit ) ] = ( int( data[ (2 * byte):((2 * byte)+2) ], 16 ) & ( 1 << bit ) ) >> bit

  rv['dBit 7'] = b7
  rv['dBit 6'] = b6
  rv['dBit 5'] = b5
  rv['dBit 4'] = b4
  rv['dBit 3'] = b3
  rv['dBit 2'] = b2
  rv['dBit 1'] = b1
  rv['dBit 0'] = b0
  rv['dBit 15'] = b15
  rv['dBit 14'] = b14
  rv['dBit 13'] = b13

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

