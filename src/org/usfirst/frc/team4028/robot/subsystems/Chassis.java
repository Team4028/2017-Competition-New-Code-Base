package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.utilities.LogData;
import org.usfirst.frc.team4028.robot.constants.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the GEAR Subsystem
//------------------------------------------------------
//	Rev		By		 	D/T			Desc
//	===		========	===========	=================================
//  1.0		Seabass		25.Feb.2017		Initial 
//------------------------------------------------------
//=====> For Changes see Sebastian Rodriguez
public class Chassis {
	// =====================================================================
	// 4 DC Motors
	//		2 Talon w/ Encoder		Left + Right Master
	//		2 Talon w/o Encoder		Left + Right Slave
	//
	// 1 Solenoid
	// 		1 Dual Action 			Shifter
	// =====================================================================
	
	// define class level variables for Robot objects
	private CANTalon _leftDriveMaster, _leftDriveSlave, _rightDriveMaster, _rightDriveSlave;
	private RobotDrive _robotDrive;				// this supports arcade/tank style drive controls
	private DoubleSolenoid _shifterSolenoid;
	
	// define class level variables to hold state
	private Value _shifterSolenoidPosition;
	private long _lastCmdChgTimeStamp;
	private double _driveSpeedScalingFactorClamped;
	
	// acc/dec variables
	private boolean _isAccelDecelEnabled;
	private double _currentThrottleCmdScaled, _previousThrottleCmdScaled;
	private double _currentThrottleCmdAccDec, _previousThrottleCmdAccDec;
	
	private double _arcadeDriveThrottleCmdAdj;
	private double _arcadeDriveTurnCmdAdj;
	
	private static final double ACC_DEC_RATE_FACTOR = 5.0;
	private static final double ACC_DEC_TOTAL_TIME_SECS = 0.8;
	
	private static final double _turnSpeedScalingFactor = 0.7;
	
	// motion magic constants
	private static final double P_GAIN = 4.0;
	private static final double I_GAIN = 0.0;
	private static final double D_GAIN = 95.0;	
	private static final double F_GAIN = 0.52;
	private static final double MAX_ACCELERATION = 1200.0; // 200 RPM / S
	private static final double MAX_VELOCITY = 150.0; // 200 RPM 
	
	// Gearbox Ratios: 1:3 encoder shaft, 34:50 output shaft
	
	// define public enums exposed by this class
	public enum GearShiftPosition {
		UNKNOWN,
		HIGH_GEAR,
		LOW_GEAR
	}	
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public Chassis(int talonLeftMasterCanBusAddr, int talonLeftSlave1CanBusAddr,
					int talonRightMasterCanBusAddr, int talonRightSlave1CanBusAddr,
					int pcmCanBusAddress, 
					int shifterSolenoidHighGearPCMPort, int shifterSolenoidLowGearPCMPort) {
    	// ===================
    	// Left Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
    	_leftDriveMaster = new CANTalon(talonLeftMasterCanBusAddr);
    	_leftDriveMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_leftDriveMaster.configEncoderCodesPerRev(1097);
    	_leftDriveMaster.reverseSensor(false);  							// do not invert encoder feedback
    	_leftDriveMaster.enableLimitSwitch(false, false);
    	_leftDriveMaster.setMotionMagicAcceleration(MAX_ACCELERATION);
    	_leftDriveMaster.setMotionMagicCruiseVelocity(MAX_VELOCITY);
    	_leftDriveMaster.setPID(P_GAIN, I_GAIN, D_GAIN);
    	_leftDriveMaster.setF(F_GAIN);

		_leftDriveSlave = new CANTalon(talonLeftSlave1CanBusAddr);
	   	_leftDriveSlave.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
	   	_leftDriveSlave.set(talonLeftMasterCanBusAddr);
	    _leftDriveSlave.enableLimitSwitch(false, false);

    	// ===================
    	// Right Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
		_rightDriveMaster = new CANTalon(talonRightMasterCanBusAddr);
    	_rightDriveMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_rightDriveMaster.configEncoderCodesPerRev(1097);
    	_rightDriveMaster.reverseSensor(true);  							// do not invert encoder feedback
    	_rightDriveMaster.reverseOutput(true);
		_rightDriveMaster.enableLimitSwitch(false, false);
		_rightDriveMaster.setMotionMagicAcceleration(MAX_ACCELERATION);
    	_rightDriveMaster.setMotionMagicCruiseVelocity(MAX_VELOCITY);
    	_rightDriveMaster.setPID(P_GAIN, I_GAIN, D_GAIN);
    	_rightDriveMaster.setF(F_GAIN);

		_rightDriveSlave = new CANTalon(talonRightSlave1CanBusAddr);
		_rightDriveSlave.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
		_rightDriveSlave.set(talonRightMasterCanBusAddr);
		_rightDriveSlave.enableLimitSwitch(false, false);
    	  	
    	//====================
    	// Shifter
    	//====================
    	_shifterSolenoid = new DoubleSolenoid(pcmCanBusAddress, shifterSolenoidHighGearPCMPort, shifterSolenoidLowGearPCMPort);
    	
    	//====================
    	// Arcade Drive
    	//====================
    	// Arcade Drive configured to drive in "2 motor per side setup, other motors follow master as slaves 
    	_robotDrive = new RobotDrive(_leftDriveMaster, _rightDriveMaster);
    	_robotDrive.setSafetyEnabled(false);
    	
    	EnableBrakeMode(false); // Disable motors on drive talons
    	EnablePercentVBusMode(); // Open loop throttle
    
    	//set default scaling factor
    	_driveSpeedScalingFactorClamped = 1.0;
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// This is the (arcade) main drive method
	public void ArcadeDrive(double newThrottleCmdRaw, double newTurnCmdRaw) {
		EnablePercentVBusMode();
		// calc scaled throttle cmds
		double newThrottleCmdScaled = newThrottleCmdRaw * _driveSpeedScalingFactorClamped;
		double newTurnCmdScaled = newTurnCmdRaw * _turnSpeedScalingFactor;
		
		// if the cmd just chg'd reset 
		if(newThrottleCmdScaled != _previousThrottleCmdScaled) {
			_previousThrottleCmdScaled = _currentThrottleCmdAccDec;
			_currentThrottleCmdScaled = newThrottleCmdScaled;
			
			_lastCmdChgTimeStamp = System.currentTimeMillis();
		}
			
		// if acc/dec mode is enabled
		if(_isAccelDecelEnabled) {
			_previousThrottleCmdAccDec = _currentThrottleCmdAccDec;
			
			//implement speed scaling
			_arcadeDriveThrottleCmdAdj = calcAccelDecelThrottleCmd(_currentThrottleCmdScaled, _previousThrottleCmdScaled, _lastCmdChgTimeStamp);
			
			_currentThrottleCmdAccDec = _arcadeDriveThrottleCmdAdj;
			
			if(Math.abs(_arcadeDriveThrottleCmdAdj - _currentThrottleCmdScaled) < 0.1) {
				_previousThrottleCmdScaled = _currentThrottleCmdScaled;
			}
		} else {
			_arcadeDriveThrottleCmdAdj = newThrottleCmdScaled;
		}
		
		_arcadeDriveTurnCmdAdj = newTurnCmdScaled;
		
		// send cmd to mtr controllers
		_robotDrive.arcadeDrive(_arcadeDriveThrottleCmdAdj, _arcadeDriveTurnCmdAdj);		
	}
	
	public void TankDrive(double leftCmd, double rightCmd) {
		EnablePercentVBusMode();
		_robotDrive.tankDrive(leftCmd, rightCmd);
	}
	
	public void SetMotionMagicTargetPosition(double leftPosition, double rightPosition) {
		EnableMotionMagicMode();
		_leftDriveMaster.set(leftPosition);
		_rightDriveMaster.set(rightPosition);
	}
	
	// stop the motors
	public void FullStop() { 
		EnableBrakeMode(true);
		ArcadeDrive(0.0, 0.0);
	}
	
	public void EnableBrakeMode(boolean isEnabled) {
		_leftDriveMaster.enableBrakeMode(isEnabled);
		_leftDriveSlave.enableBrakeMode(isEnabled);
		_rightDriveMaster.enableBrakeMode(isEnabled);
		_rightDriveSlave.enableBrakeMode(isEnabled);
	}
	
	public void EnablePercentVBusMode() {
		if (_leftDriveMaster.getControlMode() != TalonControlMode.PercentVbus) {
			_leftDriveMaster.changeControlMode(TalonControlMode.PercentVbus);
			_rightDriveMaster.changeControlMode(TalonControlMode.PercentVbus);
		}
	}
	
	public void EnableMotionMagicMode() {
		if (_leftDriveMaster.getControlMode() != TalonControlMode.MotionMagic) {
			_leftDriveMaster.changeControlMode(TalonControlMode.MotionMagic);
			_rightDriveMaster.changeControlMode(TalonControlMode.MotionMagic);
		}
	}
	
	// shifts between high & low gear
	public void ShiftGear(GearShiftPosition gear) {
		// send cmd to to solenoids
		switch(gear) {
			case HIGH_GEAR:
				_shifterSolenoid.set(RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION);
				_shifterSolenoidPosition = RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION;
				
    			DriverStation.reportWarning("Shift into HIGH gear", false);
				break;
			
			case LOW_GEAR:
				_shifterSolenoid.set(RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION);
				_shifterSolenoidPosition = RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION;
				
    			DriverStation.reportWarning("Shift into LOW gear", false);
				break;
		}
	}
	
	public void ToggleShiftGear() {
		if (_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION) {
			ShiftGear(GearShiftPosition.LOW_GEAR);
		} else {	
			ShiftGear(GearShiftPosition.HIGH_GEAR);
		}
	}
	
	public void ZeroDriveEncoders() {
		_leftDriveMaster.setPosition(0.0);
		_rightDriveMaster.setPosition(0.0);
	}
	
	// update the Dashboard with any Chassis specific data values
	public void OutputToSmartDashboard() {
		String chassisDriveGearPosition = "";
		if (_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION) {
			chassisDriveGearPosition = "HIGH_GEAR";
		} 
		else if (_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION) {
			chassisDriveGearPosition = "LOW_GEAR";
		} else {
			chassisDriveGearPosition = "UNKNOWN";
		}
		
		SmartDashboard.putString("Driving Gear", chassisDriveGearPosition);
		SmartDashboard.putNumber("Left Position", getLeftEncoderCurrentPosition());
		SmartDashboard.putNumber("Right Position", getRightEncoderCurrentPosition());
		SmartDashboard.putNumber("Left Velocity", getLeftEncoderCurrentVelocity());
		SmartDashboard.putNumber("Right Velocity", getRightEncoderCurrentVelocity());
	}
	
	public void UpdateLogData(LogData logData) {
		logData.AddData("Chassis:LeftDriveMtrSpd", String.format("%.2f", _leftDriveMaster.getSpeed()));
		logData.AddData("Chassis:LeftDriveMtr%VBus", String.format("%.2f", _leftDriveMaster.getOutputVoltage()/_leftDriveMaster.getBusVoltage()));
		logData.AddData("Chassis:LeftDriveMtrPos", String.format("%.0f", _leftDriveMaster.getPosition()));
		
		logData.AddData("Chassis:RightDriveMtrSpd", String.format("%.2f", _rightDriveMaster.getSpeed()));
		logData.AddData("Chassis:RightDriveMtr%VBus", String.format("%.2f", _rightDriveMaster.getOutputVoltage()/_rightDriveMaster.getBusVoltage()));
		logData.AddData("Chassis:RightDriveMtrPos", String.format("%.0f", _rightDriveMaster.getPosition()));
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	
	public void setDriveSpeedScalingFactor(double speedScalingFactor) {
		// for safety, clamp the scaling factor to max of +1, -1
		if (speedScalingFactor > 1.0) {
			speedScalingFactor = 1.0;
		}
		else if (speedScalingFactor < -1.0){
			speedScalingFactor = -1.0;
		}
		
		_driveSpeedScalingFactorClamped = speedScalingFactor;
	}
	
	public void setIsAccDecModeEnabled(boolean isEnabled) {
		_isAccelDecelEnabled = isEnabled;
		DriverStation.reportWarning("===== Acc/Dec Mode Enabled? " + isEnabled, false);
	}
	
	public boolean getIsAccDecModeEnabled() {
		return _isAccelDecelEnabled;
	}
	
	public double getLeftEncoderCurrentPosition() {
		return _leftDriveMaster.getPosition();
	}
	
	public double getLeftEncoderCurrentVelocity() {
		return (_leftDriveMaster.getEncVelocity()/7.5);
	}
	
	public double getRightEncoderCurrentPosition() {
		return _rightDriveMaster.getPosition();
	}
	
	public double getRightEncoderCurrentVelocity() {
		return (_rightDriveMaster.getEncVelocity()/7.5);
	}
	
	//============================================================================================
	// Utility Helper Methods
	//============================================================================================
	// implement s-curve accel / decel
	private double calcAccelDecelThrottleCmd(double currentThrottleCmd, double previousThrottleCmd, long lastCmdChgTimeStamp) {
		double accDecMidpointTimeSecs = ACC_DEC_TOTAL_TIME_SECS / 2.0;    // a

        double minusK = -1.0 * ACC_DEC_RATE_FACTOR;
        double elapsedSecsSinceLastChg = (System.currentTimeMillis() - _lastCmdChgTimeStamp) / 1000.0; // x
        double xMinusA = elapsedSecsSinceLastChg - accDecMidpointTimeSecs;

        double scaleFactor = 1.0 / ( 1.0 + Math.exp(minusK * xMinusA) );

        // finally calc the adj cmd
        double accDecCmd = previousThrottleCmd + ((_currentThrottleCmdScaled - previousThrottleCmd) * scaleFactor);
        
        return accDecCmd;
	}
}