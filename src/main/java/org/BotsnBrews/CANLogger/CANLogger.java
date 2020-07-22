package org.BotsnBrews.CANLogger;

import java.time.LocalDateTime;
import edu.wpi.first.wpilibj.CAN;

public class CANLogger {

    private static final int kDeviceId = 0;             // Assume a single CAN logger at device ID 0
    private static final int kAPIStopLogger = 0;
    private static final int kAPIStartLogger = 1;
   
    //private CAN mCanLogger;
    private LocalDateTime mLocalDateTime;
    private byte[] mFrameData = {0, 0, 0, 0, 0, 0, 0, 0};

    /**
    * This method will stop the CAN logger.
    */    
    public void StopLogging () {
        //mCanLogger.writePacket( mFrameData, kAPIStopLogger );
    }

    /**
    * This method will start the CAN logger and the resulting filename on the
    * CAN logger micro SD will be of the format YYYY-MM-DD_HH:MM:SS.bin
    */    
    public void StartLogging () {
        mLocalDateTime = LocalDateTime.now();
        mFrameData[0] = (byte) ( mLocalDateTime.getYear() - 2000 );
        mFrameData[1] = (byte) mLocalDateTime.getMonthValue();
        mFrameData[2] = (byte) mLocalDateTime.getDayOfMonth();
        mFrameData[3] = (byte) mLocalDateTime.getHour();
        mFrameData[4] = (byte) mLocalDateTime.getMinute();
        mFrameData[5] = (byte) mLocalDateTime.getSecond();
        //mCanLogger.writePacket( mFrameData, kAPIStartLogger );
    }

    /**
    * This is the CANLogger class consructor.
    */    
    public CANLogger () {
        //mCanLogger = new CAN( kDeviceId );
    }

}
