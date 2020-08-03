#-------------------------------------------------------------------------------
def twosComp ( intVal, numBits ):
  """

  This function will compute the two's compliment for the given unsigned integer
  value and bit length.

  Parameters
  ----------
  intVal:
    An unsigned integer value     
  numBits:
    Bit length of the input integer (register) 

  Returns
  -------
  signedIntVal:
    Signed two's compliment integer

  """
  signedIntVal = intVal
  if ( signedIntVal & ( 1 << ( numBits - 1 ) ) ) != 0:
    rv = signedIntVal - ( 1 << numBits )
  return signedIntVal


#-------------------------------------------------------------------------------
def hex8ByteTo64BitVal ( hex2BinaryVal ):
  """

  This function will convert the given hex string to a binary string.

  Parameters
  ----------
  hex8ByteVal:
    An 8-byte hex string
  
  Returns
  -------
  bin64BitVal:
    A 64-bit binary string

  """
  bin64BitVal = ( bin( int( hex8ByteVal[  0:2  ], 16 ) )[ 2: ].zfill( 8 ) +
                  bin( int( hex8ByteVal[  2:4  ], 16 ) )[ 2: ].zfill( 8 ) +
                  bin( int( hex8ByteVal[  4:6  ], 16 ) )[ 2: ].zfill( 8 ) +
                  bin( int( hex8ByteVal[  6:8  ], 16 ) )[ 2: ].zfill( 8 ) +
                  bin( int( hex8ByteVal[  8:10 ], 16 ) )[ 2: ].zfill( 8 ) +
                  bin( int( hex8ByteVal[ 10:12 ], 16 ) )[ 2: ].zfill( 8 ) +
                  bin( int( hex8ByteVal[ 12:14 ], 16 ) )[ 2: ].zfill( 8 ) +
                  bin( int( hex8ByteVal[ 14:16 ], 16 ) )[ 2: ].zfill( 8 ) )
  return bin64BitVal


#-------------------------------------------------------------------------------
class FrameTracker ():
  """

  This class is used to track a CAN frame data and aid in determining the
  meanings of the various CAN frame data by tracking the total and
  frame-to-frame bit changes.  When creating an object of this class an
  initial value needs to be provided as a baseline for the cumulative
  bit monitoring and seeding the frame-to-frame monitoring.

  Methods
  -------
    updateBitChanges( hex8ByteVal ):
      This method will update the tracked frames bit changes with respect to the
      prior frame and the frame data used during object construction.

    getCumulativeChangingBits():
      This method will return a string indicating the bits which have changed
      anytime during the updateBitChanges() calls.

    getPriorChangingBits():
      This method will return a string indicating the bits which have changed
      from the prior call to updateBitChanges().

  Attributes
  ----------

  """
  def __init__ ( self, hexVal, bitLength ):
    self.length = bitLength
    self.binaryCumChanges = '1' * bitLength
    self.binaryPrevChanges = '1' * bitLength
    self.baseBinaryVal = hex2BinaryVal( hexVal )
    self.baseBinaryPrevVal = hex2BinaryVal( hexVal )


  def _getChangingBits ( self, baseBits, newBits ):
    # Find the matching 0's
    m_0s = bin( int( ''.join( '1' if x == '0' else '0' for x in baseBits ), 2 ) &
                int( ''.join( '1' if x == '0' else '0' for x in newBits ), 2 ) )[ 2: ].zfill( self.length )
    # Find the matching 1's
    m_1s = bin( int( baseBits, 2 ) &
                int( newBits, 2 ) )[2:].zfill( self.length )
    # Bit changes are 0's
    bitChangeMask = bin( int( m_0s, 2 ) | int( m_1s, 2 ) )[ 2: ].zfill( self.length )
    return bitChangeMask

    
  def updateBitChanges ( self, hex8ByteVal ):
    """
    This method will update the attributes which track the total and
    frame-to-frame bit changes.  This should be called whenever a new data for a
    frame is available.

    Parameters
    ----------
      hex8ByteVal:
        An 8-byte hex string

    Returns
    -------
      none
    """
    bin64BitVal = hex8ByteTo64BitVal( hex8ByteVal )

    # Track the overall bit changes
    bitChangeMask = self._getChangingBits( self.baseBin64BitVal, bin64BitVal )
    self.bin64BitCumulativeChanges = bin( int( bitChangeMask, 2 ) &
                                          int( self.bin64BitCumulativeChanges, 2 ) )[ 2: ].zfill( 64 )
    
    # Track the changes from the prior
    self.bin64BitPreviousChanges = self._getChangingBits( self.baseBin64BitPreviousVal, bin64BitVal )
    self.baseBin64BitPreviousVal = bin64BitVal



  def getCumulativeChangingBits ( self ):
    """
    This method returns a string a indicating the bits which have changed 
    anytime during the updateBitChanges() calls

    Parameters
    ----------
      none

    Returns
    -------
    rv:
      A string indicating which bits have changed
    """
    rv = ''
    for i, v in enumerate( self.bin64BitCumulativeChanges ):
      if ( v == '0' ):
        rv = rv + 'BIT' + str( i )
    return rv


  def getPriorChangingBits ( self ):
    """
    This method returns a string a indicating the bits which have changed from
    the prior call to updateBitChanges().

    Parameters
    ----------
      none

    Returns
    -------
    rv:
      A string indicating which bits have changed
    """
    rv = ''
    for i, v in enumerate( self.bin64BitPreviousChanges ):
      if ( v == '0' ):
        rv = rv + 'BIT' + str( i )
    return rv        
            
