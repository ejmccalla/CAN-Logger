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
    signedIntVal = signedIntVal - ( 1 << numBits )
  return signedIntVal


#-------------------------------------------------------------------------------
def hex2Bin ( hexVal ):
  """

  This function will convert the given hex string to a binary string.

  Parameters
  ----------
  hexVal:
    An hex string without the leading '0x'
  
  Returns
  -------
  binVal:
    A binary string without the leading '0b'

  """
  binVal = ''.join( bin( int( val, 16 ) )[ 2: ].zfill( 4 ) for val in hexVal )
  return binVal


#-------------------------------------------------------------------------------
def flipBits ( binVal ):
  """

  This function will convert the given binary string to a binary string with the
  bits flipped.

  Parameters
  ----------
  binVal:
    An binary string without the leading '0b'
  
  Returns
  -------
  flippedBinVal:
    A binary string of flipped bits without the leading '0b'

  """
  flippedBinVal = ''.join( '1' if val == '0' else '0' for val in binVal )
  return flippedBinVal


#-------------------------------------------------------------------------------
class BitTracker ():
  """

  This class is used to track a CAN frame data and aid in determining the
  meanings of the various CAN frame data by tracking the total and the
  frame-to-frame bit changes.  When creating an object of this class an
  initial value needs to be provided as a baseline for the cumulative
  bit monitoring and seeding the frame-to-frame monitoring as well as the bit
  length of the data to monitor.

  Attributes
  ----------
    bitLength:
      The number of bits being tracked.  All hex values passed to this class
      will perform conversions based on this.

  Methods
  -------
    updateBitChanges( hexVal ):
      This method will update the tracked frames bit changes with respect to the
      prior frame and the frame data used during object construction.

    getCumulativeChangingBits():
      This method will return a string indicating the bits which have changed
      anytime during the updateBitChanges() calls.

    getPriorChangingBits():
      This method will return a string indicating the bits which have changed
      from the prior call to updateBitChanges().

  """
  def __init__ ( self, hexVal, bitLength ):
    self.bitLength = bitLength
    self.binCumChanges = '1' * bitLength
    self.binPrevChanges = '1' * bitLength
    self.baseBinVal = hex2Bin( hexVal )
    self.baseBinPrevVal = hex2Bin( hexVal )


  def _getChangingBits ( self, baseBits, newBits ):
    """
    This method will determing the changing bits between the provided inputs.

    Parameters
    ----------
      baseBits:
        A binary string without the leading '0b'
      newBits:
        A binary string without the leading '0b'

    Returns
    -------
      bitChangeMask:
        A binary string where 0's indicate a changing bit and 1's indicate bits
        that are unchanged.
    """    
    # Find the matching 0's
    m_0s = bin( int( flipBits( baseBits ), 2 ) &
                int( flipBits( newBits ), 2 ) )[ 2: ].zfill( self.bitLength )
    # Find the matching 1's
    m_1s = bin( int( baseBits, 2 ) &
                int( newBits, 2 ) )[2:].zfill( self.bitLength )
    # Bit changes are 0's
    bitChangeMask = bin( int( m_0s, 2 ) |
                         int( m_1s, 2 ) )[ 2: ].zfill( self.bitLength )
    return bitChangeMask

    
  def updateBitChanges ( self, hexVal ):
    """
    This method will update the attributes which track the total and
    frame-to-frame bit changes.  This should be called whenever a new data for a
    frame is available.

    Parameters
    ----------
      hexVal:
        A hex string without the leading '0x'

    Returns
    -------
      none
    """
    binVal = hex2Bin( hexVal )

    # Track the overall bit changes
    bitChangeMask = self._getChangingBits( self.baseBinVal, binVal )
    self.binCumChanges = bin( int( bitChangeMask, 2 ) &
                              int( self.binCumChanges, 2 ) )[ 2: ].zfill( 64 )
    
    # Track the changes from the prior
    self.binPrevChanges = self._getChangingBits( self.baseBinPrevVal, binVal )
    self.baseBinPrevVal = binVal


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
    for i, v in enumerate( self.binCumChanges ):
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
    for i, v in enumerate( self.binPrevChanges ):
      if ( v == '0' ):
        rv = rv + 'BIT' + str( i )
    return rv        
            
