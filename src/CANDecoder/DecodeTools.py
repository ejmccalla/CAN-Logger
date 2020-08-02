class DecodeTools ():

    def __init__ ( self, hex8ByteData ):
        self.bin64BitTotChanges = '1111111111111111111111111111111111111111111111111111111111111111'
        self.bin64BitPrevChanges = '1111111111111111111111111111111111111111111111111111111111111111'
        self.baseBin64BitData = self.HexTo64Bit( hex8ByteData )
        self.baseBin64BitPrevData = self.HexTo64Bit( hex8ByteData )

    def TwosComp ( self, intData, numBits ):
        rv = intData
        if ( intData & ( 1 << ( numBits - 1 ) ) ) != 0:
            rv = intData - ( 1 << numBits )
        return rv

    def HexTo64Bit ( self, hex8ByteData ):
        bin64BitData = ( bin( int( hex8ByteData[  0:2  ], 16 ) )[ 2: ].zfill( 8 ) +
                         bin( int( hex8ByteData[  2:4  ], 16 ) )[ 2: ].zfill( 8 ) +
                         bin( int( hex8ByteData[  4:6  ], 16 ) )[ 2: ].zfill( 8 ) +
                         bin( int( hex8ByteData[  6:8  ], 16 ) )[ 2: ].zfill( 8 ) +
                         bin( int( hex8ByteData[  8:10 ], 16 ) )[ 2: ].zfill( 8 ) +
                         bin( int( hex8ByteData[ 10:12 ], 16 ) )[ 2: ].zfill( 8 ) +
                         bin( int( hex8ByteData[ 12:14 ], 16 ) )[ 2: ].zfill( 8 ) +
                         bin( int( hex8ByteData[ 14:16 ], 16 ) )[ 2: ].zfill( 8 ) )
        return bin64BitData

    def ComputeChangingBits ( self, baseBits, newBits ):
        # Find the matching 0's
        mathchingZeros = bin( int( ''.join( '1' if x == '0' else '0' for x in baseBits ), 2 ) &
                              int( ''.join( '1' if x == '0' else '0' for x in newBits ), 2 ) )[ 2: ].zfill( 64 )
        # Find the matching 1's
        mathchingOnes = bin( int( baseBits, 2 ) &
                             int( newBits, 2 ) )[2:].zfill( 64 )
        # Bit changes are 0's
        bitChangeMask = bin( int( mathchingZeros, 2 ) |
                             int( mathchingOnes, 2 ) )[ 2: ].zfill( 64 )
        return bitChangeMask

    
    def UpdateBitChanges ( self, hex8ByteData):
        bin64BitData = self.HexTo64Bit( hex8ByteData )

        # Track the overall bit changes
        bitChangeMask = self.ComputeChangingBits( self.baseBin64BitData, bin64BitData )
        self.bin64BitTotChanges = bin( int( bitChangeMask, 2 ) & int( self.bin64BitTotChanges, 2 ) )[ 2: ].zfill( 64 )
        
        # Track the changes from the prior
        self.bin64BitPrevChanges = self.ComputeChangingBits( self.baseBin64BitPrevData, bin64BitData )
        self.baseBin64BitPrevData = bin64BitData


    def GetCumulativeChangingBits ( self ):
        rv = ''
        for i, v in enumerate( self.bin64BitTotChanges ):
            if ( v == '0' ):
                rv = rv + 'BIT' + str( i )
        return rv

    def GetPriorChangingBits ( self ):
        rv = ''
        for i, v in enumerate( self.bin64BitPrevChanges ):
            if ( v == '0' ):
                rv = rv + 'BIT' + str( i )
        return rv        
            
