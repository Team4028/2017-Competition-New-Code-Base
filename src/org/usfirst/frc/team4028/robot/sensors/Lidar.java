package org.usfirst.frc.team4028.robot.sensors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.TimerTask;

import org.usfirst.frc.team4028.robot.utilities.LogData;

import java.lang.String;

public class Lidar {
	private final int TEXT_CHUNK_MIN_LENGTH = 9;
	private final int LIDAR_TIMEOUT_IN_MSEC = 250;
	private final int DISCONNECT_TIMER_IN_MSEC = 10000;
	
	private java.util.Timer _updaterTimer;
	private LIDARUpdaterTask _task;
	private boolean _isRunning = false;
	private boolean _isDataValid = false;
	private  int _distance;
	private long _lastTimeoutMessage;
	SerialPort _serialport;
	String _rawDistanceText;
	long _lastGoodDataRead;
	
	//=========================================================================
	//	Constructor(s)
	//=========================================================================
	public Lidar(SerialPort.Port PortConstant) {
		this(PortConstant, 20);
	}

	public Lidar(SerialPort.Port PortConstant, int period) {
		_task = new LIDARUpdaterTask();
		_serialport = new SerialPort(9600, PortConstant);
		_updaterTimer = new java.util.Timer();
		_updaterTimer.scheduleAtFixedRate(_task, 0, period);
		_lastTimeoutMessage = System.currentTimeMillis();
	}
	
	//=========================================================================
	//	Methods
	//=========================================================================
	
	public void start() {	// START REGULAR DISTANCE POLLING
		_isRunning = true;
	}
	
	public void stop() {			//	STOP REGULAR DISTANCE POLLING
		_isRunning = false;
	}
	
	public void update() {		// UPDATE DISTANCE VAR (called regularly by LIDARUpdaterTask)
		String readString = _serialport.readString();
		if(System.currentTimeMillis() - _lastGoodDataRead > LIDAR_TIMEOUT_IN_MSEC) {
			// update() has not received valid, interpretable data recently
			_isDataValid = false;
			if(System.currentTimeMillis() - _lastTimeoutMessage > DISCONNECT_TIMER_IN_MSEC) {
				// Time since last Lidar timeout message is past threshold
				DriverStation.reportError("LIDAR TIMEOUT", false);
				_lastTimeoutMessage = System.currentTimeMillis();
			}
		}
			
		if(!readString.isEmpty()) {
			// readString is not empty
			_rawDistanceText = _rawDistanceText + readString;
			if (_rawDistanceText.length() >= TEXT_CHUNK_MIN_LENGTH) {
				// String length is guaranteed long enough to contain a full measurement read
				// (not guaranteed to be uncorrupted)
				int hashtagLocation = _rawDistanceText.indexOf("#");
				String justDistanceText = _rawDistanceText.substring(hashtagLocation + 1, hashtagLocation + 5);
				try {
					_distance = Integer.parseInt(justDistanceText);
					_lastGoodDataRead = System.currentTimeMillis();
					_isDataValid = true;
				}
				catch (NumberFormatException e) {
					// this exception is thrown if Integer.parseInt() receives a string that is not interpretable as an integer
					DriverStation.reportError("Bad LIDAR data", false);
				}
				// Clear the current data out of _RawDistanceText
				_rawDistanceText = "";
			}
		}
	}
	
	// update the Dashboard with any Climber specific data values
	public void OutputToSmartDashboard() {
		SmartDashboard.putBoolean("Lidar:IsValid", get_IsValid());
		SmartDashboard.putString("Lidar:DistanceInCm", String.format("%01d", get_DistanceInCm()));
	}
	
	public void UpdateLogData(LogData logData) {
		logData.AddData("Lidar:DistanceInCm", String.format("%01d", get_DistanceInCm()));
		logData.AddData("Lidar:IsValid", Boolean.toString(get_IsValid()));
	}
		
	//=========================================================================
	//	Properties
	//=========================================================================	
	public int get_DistanceInCm () {
		return _distance;
	}

	public boolean get_IsValid () {
		// indicates that data has been received and correctly interpreted as an int within the LIDAR_TIMEOUT threshold
		return _isDataValid;
	}
	
	//=========================================================================
	//	Task Executed By Timer
	//=========================================================================	
	private class LIDARUpdaterTask extends TimerTask {
		public void run() {
			while (_isRunning) {
				update();

				try {
					Thread.sleep(10); //10 MS
				}
				catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}