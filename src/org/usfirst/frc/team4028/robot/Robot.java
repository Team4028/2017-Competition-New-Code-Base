package org.usfirst.frc.team4028.robot;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Date;

import org.usfirst.frc.team4028.robot.autonRoutines.CrossBaseLine;
import org.usfirst.frc.team4028.robot.autonRoutines.DoNothing;
import org.usfirst.frc.team4028.robot.autonRoutines.HangBoilerGear;
import org.usfirst.frc.team4028.robot.autonRoutines.HangBoilerGearAndShoot;
import org.usfirst.frc.team4028.robot.autonRoutines.HangCenterGear;
import org.usfirst.frc.team4028.robot.autonRoutines.HangCenterGearAndShoot;
import org.usfirst.frc.team4028.robot.autonRoutines.HangRetrievalGear;
import org.usfirst.frc.team4028.robot.autonRoutines.HitHopper;
import org.usfirst.frc.team4028.robot.autonRoutines.TurnAndShoot;
import org.usfirst.frc.team4028.robot.autonRoutines.TwoGear;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ALLIANCE_COLOR;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.AUTON_MODE;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.TELEOP_MODE;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.VISION_CAMERAS;
import org.usfirst.frc.team4028.robot.controllers.AutoShootController;
import org.usfirst.frc.team4028.robot.controllers.ChassisAutoAimController;
import org.usfirst.frc.team4028.robot.controllers.HangGearController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.sensors.Lidar;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.sensors.RoboRealmClient;
import org.usfirst.frc.team4028.robot.sensors.SwitchableCameraServer;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.robot.utilities.DataLogger;
import org.usfirst.frc.team4028.robot.utilities.LogData;
import org.usfirst.frc.team4028.robot.utilities.MovingAverage;
import org.usfirst.frc.team4028.robot.utilities.ShooterTable;
import org.usfirst.frc.team4028.robot.utilities.GeneralUtilities;
import org.usfirst.frc.team4028.robot.subsystems.Climber;
import org.usfirst.frc.team4028.robot.subsystems.DashboardInputs;
import org.usfirst.frc.team4028.robot.subsystems.DriversStation;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.BallInfeed;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The is the main code for:
 * 	    Team:	4028 "The Beak Squad"
 * 		Season: FRC 2017 "First Steamworks"
 * 		Robot:	Competition Chassis
 */
public class Robot extends IterativeRobot {
	// this value is printed on the Driver's Station message window on startup
	private static final String ROBOT_NAME = "COMPETITION Chassis";
	
	// ===========================================================
	//   Define class level instance variables for Robot Runtime objects  
	// ===========================================================
	private Chassis _chassis;
	private Climber _climber;
	private BallInfeed _ballInfeed;
	private GearHandler _gearHandler;
	private Shooter _shooter;
	
	private DashboardInputs _dashboardInputs;
	private DriversStation _driversStation;
	
	// sensors
	private Lidar _lidar;
	private NavXGyro _navX;
	private SwitchableCameraServer _switchableCameraServer;
	private RoboRealmClient _roboRealmClient;
	
	// Wrapper around data logging (will be null if logging is not enabled)
	private DataLogger _dataLogger;
	
	// ===========================================================
	//   Define class level instance variables for Robot State
	// ===========================================================
	private TELEOP_MODE _teleopMode;
	private AUTON_MODE _autonMode;
	private ALLIANCE_COLOR _allianceColor;
	
	// ===========================================================
	//   Define class level instance variables for Robot Controllers
	// ===========================================================
	private AutoShootController _autoShootController;
	private ChassisAutoAimController _chassisAutoAimGyro;
	private ChassisAutoAimController _chassisAutoAimVision;
	private HangGearController _hangGearController;
	private TrajectoryDriveController _trajController;
	
	// ===========================================================
	//   Define class level instance variables for Robot Auton Routines 
	// ===========================================================
	CrossBaseLine _crossBaseLineAuton;
	DoNothing _doNothingAuton;
	HangBoilerGear _hangBoilerGearAuton;
	HangBoilerGearAndShoot _hangBoilerGearAndShootAuton;
	HangCenterGear _hangCenterGearAuton;
	HangCenterGearAndShoot _hangCenterGearAndShootAuton;
	HangRetrievalGear _hangRetrievalGear;
	HitHopper _hitHopper;
	TurnAndShoot _turnAndShoot;
	TwoGear _twoGearAuton;
	
	// ===========================================================
	//   Define class level working variables
	// ===========================================================
	String _buildMsg = "?";
	ShooterTable _shooterTable;
	String _fmsDebugMsg = "?";
 	long _lastDashboardWriteTimeMSec;
 	long _lastScanEndTimeInMSec;
 	MovingAverage _scanTimeSamples;
	
	// --------------------------------------------------------------------------------------------------------------------------------------------
	// Code executed 1x at robot startup																		ROBOT INIT
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void robotInit() {
        //===================
    	// write jar (build) date & time to the dashboard
        //===================
		_buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
    	
        //===================
    	// create instances (and configure) all of all robot subsystems & sensors
        //===================
		_ballInfeed = new BallInfeed(RobotMap.BALL_FLOOR_INFEED_MTR_CAN_BUS_ADDR, 
										RobotMap.PCM_CAN_BUS_ADDR, 
										RobotMap.BALL_FLOOR_INFEED_EXTEND_PCM_PORT);
		
		_chassis = new Chassis(RobotMap.LEFT_DRIVE_MASTER_CAN_BUS_ADDR, 
								RobotMap.LEFT_DRIVE_SLAVE1_CAN_BUS_ADDR, 
								RobotMap.RIGHT_DRIVE_MASTER_CAN_BUS_ADDR, 
								RobotMap.RIGHT_DRIVE_SLAVE1_CAN_BUS_ADDR,
								RobotMap.PCM_CAN_BUS_ADDR,
								RobotMap.SHIFTER_SOLENOID_EXTEND_PCM_PORT,
								RobotMap.SHIFTER_SOLENOID_RETRACT_PCM_PORT);
		
		_climber = new Climber(RobotMap.CLIMBER_CAN_BUS_ADDR);
		
		_dashboardInputs = new DashboardInputs();
		
		_driversStation = new DriversStation(RobotMap.DRIVER_GAMEPAD_USB_PORT, 
												RobotMap.OPERATOR_GAMEPAD_USB_PORT,
												RobotMap.ENGINEERING_GAMEPAD_USB_PORT);
	
		_gearHandler = new GearHandler(RobotMap.GEAR_TILT_CAN_BUS_ADDR, RobotMap.GEAR_INFEED_CAN_BUS_ADDR);
		
		_shooterTable = new ShooterTable();
		
		_shooter = new Shooter(RobotMap.SHOOTER_STG1_CAN_BUS_ADDR, 
								RobotMap.SHOOTER_STG2_CAN_BUS_ADDR,
								RobotMap.MAGIC_CARPET_CAN_BUS_ADDR,
								RobotMap.HIGH_SPEED_INFEED_LANE_CAN_BUS_ADDR,
								RobotMap.HIGH_ROLLER_CAN_BUS_ADDR,
								RobotMap.SHOOTER_SLIDER_PWM_PORT,
								_shooterTable);
		
		// sensors follow
		//_lidar = new Lidar(SerialPort.Port.kMXP);		// TODO: Re-enable?
		_navX = new NavXGyro(RobotMap.NAVX_PORT);
		
		_switchableCameraServer = new SwitchableCameraServer("cam0");			//safe
		_roboRealmClient = new RoboRealmClient(RobotMap.KANGAROO_IPV4_ADDR, 
												RobotMap.RR_API_PORT,
												RobotMap.LED_RINGS_DIO_PORT); 
												
												//RobotMap.PCM_CAN_BUS_ADDR,
												//RobotMap.GEAR_LED_RING_PCM_PORT,
												//RobotMap.BOILER_LED_RING_PCM_PORT);
		
		// telop Controller follow
		_chassisAutoAimGyro = new ChassisAutoAimController(_chassis, _navX, 0.05, 0.0, 0.0);
		_chassisAutoAimVision = new ChassisAutoAimController(_chassis, _navX, 0.065, 0.0075, 0.0);
		_autoShootController = new AutoShootController(_chassisAutoAimVision, _roboRealmClient, _shooter, _shooterTable);
		_hangGearController = new HangGearController(_gearHandler, _chassis);
		_trajController = new TrajectoryDriveController(_chassis, _navX, _roboRealmClient);
				
		// debug info for FMS Alliance sensing
		boolean isFMSAttached = _dashboardInputs.getIsFMSAttached();
		ALLIANCE_COLOR allianceColor = _dashboardInputs.get_allianceColor();
		_fmsDebugMsg = "Is FMS Attached: [" + isFMSAttached + "] Alliance: [" + allianceColor + "]";
		DriverStation.reportWarning(">>>>> " + _fmsDebugMsg + " <<<<<<", false);
		
		// create class to hold Scan Times moving Average samples
		_scanTimeSamples = new MovingAverage(100);  // 2 sec * 1000mSec/Sec / 20mSec/Scan
		SmartDashboard.putString("Scan Time (2 sec roll avg)", "0.0 mSec");
		
		//Update Dashboard Fields (push all fields to dashboard)
		OutputAllToSmartDashboard();
	}
	
	// --------------------------------------------------------------------------------------------------------------------------------------------
	// called each time the robot enters disabled mode from either telop or auton mode							DISABLED PERIODIC
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void disabledPeriodic() {

		// if logging was enabled, make sure we close the file
    	if(_dataLogger != null) {
	    	_dataLogger.close();
	    	_dataLogger = null;
    	}
    	
    	// stop lidar polling
    	if(_lidar != null) { 
    		_lidar.stop();
    	}
    	
    	// cleanup auton resources, since they are not needed in Telop Mode
    	if(_crossBaseLineAuton != null) {
    		_crossBaseLineAuton.Disabled();
    		_crossBaseLineAuton = null;
    	}
    	
		if(_doNothingAuton != null) {
			_doNothingAuton = null;
    	}
		
		if(_hangBoilerGearAuton != null) {
			_hangBoilerGearAuton.Disabled();
			_hangBoilerGearAuton = null;
    	}
		
		if(_hangBoilerGearAndShootAuton != null) {
			_hangBoilerGearAndShootAuton.Disabled();
			_hangBoilerGearAndShootAuton = null;
		}
		
		if(_hangCenterGearAuton != null) {
			_hangCenterGearAuton.Disabled();
			_hangCenterGearAuton = null;
    	}
		
		if(_hangCenterGearAndShootAuton != null) {
			_hangCenterGearAndShootAuton.Disabled();
			_hangCenterGearAndShootAuton = null;
    	}
		
		if(_hangRetrievalGear != null) {
			_hangRetrievalGear.Disabled();
			_hangRetrievalGear = null;
    	}
		
		if(_hitHopper != null) {
			_hitHopper.Disabled();
			_hitHopper = null;
		}
		
		if(_turnAndShoot != null) {
			_turnAndShoot.Disabled();
			_turnAndShoot = null;
    	}
		
		if(_twoGearAuton != null) {
			_twoGearAuton.Disabled();
			_twoGearAuton = null;
		}
		
		if(_roboRealmClient != null) {
			_roboRealmClient.TurnAllVisionLEDsOff();
		}
	}
		
	// --------------------------------------------------------------------------------------------------------------------------------------------
	// code executed 1x when entering AUTON Mode																AUTONOMOUS INIT
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void autonomousInit() {
		// =====================================
    	// Step 1: pre-state cleanup
		// =====================================
		
		// stop gear
    	_gearHandler.FullStop();
    	_gearHandler.ZeroGearTiltAxisInit();
    	
    	_hangGearController.setIsChassisControlEnabled(false);
    	
    	_trajController.startTrajectoryController();
    	
    	// start the lidar polling
    	if(_lidar != null)	{
    		_lidar.start(); 
    	}	
    	
    	// =====================================
		// Step 2: Read from Dashboard Choosers to select the Auton routine to run
    	// =====================================
    	_allianceColor = _dashboardInputs.get_allianceColor();
    	
    	_autonMode = _dashboardInputs.get_autonMode();

    	// =====================================
		// Step 2.1: Create the correct auton routine
    	//				since we have quite a few auton routines we only create the one we need
    	//				NOTE: each "real" auton routine is responsible to call the ZeroGearTiltAxisReentrant
    	//						as many times as required
    	// =====================================
    	switch (_autonMode) {
			case CROSS_BASE_LINE:
				_crossBaseLineAuton = new CrossBaseLine(_gearHandler, _trajController);
				_crossBaseLineAuton.Initialize();
				break;
				
			case DO_NOTHING:
				_doNothingAuton = new DoNothing();
				_doNothingAuton.Initialize();
				break;
				
			case HANG_BOILER_GEAR:
				_hangBoilerGearAuton = new HangBoilerGear(_gearHandler, _hangGearController, _trajController);
				_hangBoilerGearAuton.Initialize();
				break;
			
			case HANG_BOILER_GEAR_AND_SHOOT:
				_hangBoilerGearAndShootAuton = new HangBoilerGearAndShoot(_autoShootController, _gearHandler, _hangGearController, _shooter, _trajController, _allianceColor);
				_hangBoilerGearAndShootAuton.Initialize();
				break;
				
			case HANG_CENTER_GEAR:
				_hangCenterGearAuton = new HangCenterGear(_gearHandler, _hangGearController, _trajController);
				_hangCenterGearAuton.Initialize();
				break;
				
			case HANG_CENTER_GEAR_AND_SHOOT:
				_hangCenterGearAndShootAuton = new HangCenterGearAndShoot(_autoShootController, _gearHandler, _chassisAutoAimGyro, _hangGearController, _shooter, _trajController, _allianceColor);
				_hangCenterGearAndShootAuton.Initialize();
				break;
				
			case HANG_RETRIEVAL_GEAR:
				_hangRetrievalGear = new HangRetrievalGear(_gearHandler, _hangGearController, _trajController);
				_hangRetrievalGear.Initialize();
				break;
				
			case HIT_HOPPER:
				_hitHopper = new HitHopper(_autoShootController, _chassis, _chassisAutoAimGyro, _gearHandler, _shooter, _trajController, _allianceColor);
				_hitHopper.Initialize();
				break;
				
			case TURN_AND_SHOOT:
				_turnAndShoot = new TurnAndShoot(_autoShootController, _gearHandler, _shooter, _trajController);
				_turnAndShoot.Initialize();
				break;
				
			case TWO_GEAR:
				_twoGearAuton = new TwoGear(_gearHandler, _chassisAutoAimGyro, _hangGearController, _trajController);
				_twoGearAuton.Initialize();
				break;
				
			case UNDEFINED:
			default:
				DriverStation.reportError("Error... NO AUTON Selected!", false);
				break;
    	}
		
    	// =====================================
    	// Step 2.3: Turn all vision LEDs offg
    	// =====================================
    	_roboRealmClient.TurnAllVisionLEDsOff();
    	
       	//=========================================================
    	//Step 2.3
    	//Set the Proper Cameras for this match
    	//==========================================================
    	/*_switchableCameraServer.setGearCameraName(_dashboardInputs.get_gearCam().get_cameraName());
    	_switchableCameraServer.setShooterCameraName(_dashboardInputs.get_shooterCam().get_cameraName());
    	_switchableCameraServer.setClimberCameraName(_dashboardInputs.get_climberCam().get_cameraName());
    	_switchableCameraServer.setDriverCameraName(_dashboardInputs.get_driverCam().get_cameraName());*/
    	
    	// =====================================
    	// Step 3: Optionally Configure Logging
    	// =====================================
    	_dataLogger = GeneralUtilities.setupLogging("auton");
    	
    	_lastDashboardWriteTimeMSec = new Date().getTime();
	}

	// --------------------------------------------------------------------------------------------------------------------------------------------
	// Code executed every scan cycle (about every 20 mSec or 50x / sec) in AUTON mode							AUTONOMOUS PERIODIC
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void autonomousPeriodic() {
		// =======================================
		// if not complete, this must run concurrently with all auton routines
		// =======================================
      	if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      		//			we must treat it as a Reentrant function
      		//			and automatically recall it until complete
    		_gearHandler.ZeroGearTiltAxisReentrant();
    	}
      	  	
      	// =======================================
		// Step 2: call the correct Rentrant Auton routine to run
      	// =======================================
    	switch (_autonMode) {
			case CROSS_BASE_LINE:
				if(_crossBaseLineAuton.getIsStillRunning()) {
					_crossBaseLineAuton.ExecuteRentrant();
				}
				break;
				
			case DO_NOTHING:
				if(_doNothingAuton.getIsStillRunning()) {
					_doNothingAuton.ExecuteRentrant();
				}
				break;
				
			case HANG_BOILER_GEAR:
				if(_hangBoilerGearAuton.getIsStillRunning()) {
					_hangBoilerGearAuton.ExecuteRentrant();
				}
				break;
				
			case HANG_BOILER_GEAR_AND_SHOOT:
				if(_hangBoilerGearAndShootAuton.getIsStillRunning()) {
					_hangBoilerGearAndShootAuton.ExecuteRentrant();
				}
				break;
				
			case HANG_CENTER_GEAR:
				if(_hangCenterGearAuton.getIsStillRunning()) {
					_hangCenterGearAuton.ExecuteRentrant();
				}
				break;
				
			case HANG_CENTER_GEAR_AND_SHOOT:
				if(_hangCenterGearAndShootAuton.getIsStillRunning()) {
					_hangCenterGearAndShootAuton.ExecuteRentrant();
				}
				break;
				
			case HANG_RETRIEVAL_GEAR:
				if(_hangRetrievalGear.getIsStillRunning()) {
					_hangRetrievalGear.ExecuteRentrant();
				}
				break;
				
			case HIT_HOPPER:
				if(_hitHopper.getIsStillRunning()) {
					_hitHopper.ExecuteRentrant();
				}
				break;
				
			case TURN_AND_SHOOT:
				if(_turnAndShoot.getIsStillRunning()) {
					_turnAndShoot.ExecuteRentrant();
				}
				break;
				
			case TWO_GEAR:
				if(_twoGearAuton.getIsStillRunning()) {
					_twoGearAuton.ExecuteRentrant();
				}
				break;
				
			case UNDEFINED:
			default:
				DriverStation.reportError("Error... NO AUTON Selected!", false);
				break;
    	}
		
    	// =====================================
    	// Step 3: Optionally Log Data
    	// =====================================
		WriteLogData();
		
		OutputAllToSmartDashboard();
	}

	// --------------------------------------------------------------------------------------------------------------------------------------------
	// code executed 1x when entering TELOP Mode																TELOP INIT
	// --------------------------------------------------------------------------------------------------------------------------------------------

	@Override
	public void teleopInit() {
    	// =====================================
    	// Step 1: Setup Robot Defaults
    	// =====================================
		// #### Chassis ####
    	_chassis.FullStop(); 								// Stop Motors
    	_chassis.ZeroDriveEncoders(); 						// Zero drive encoders
    	_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);    // Set shifter to HIGH gear
    	_chassis.setIsAccDecModeEnabled(true);				// Disable acc/dec mode
		_chassis.setDriveSpeedScalingFactor(1.0);
		_chassis.EnablePercentVBusMode();
    	
    	// #### Climber ####
    	_climber.FullStop();
    	
    	// #### GearHandler ####
    	_gearHandler.FullStop();
    	if(!_gearHandler.hasTiltAxisBeenZeroed()) { 
    		_gearHandler.ZeroGearTiltAxisInit(); 
    	} else {
    		_gearHandler.MoveGearToScorePosition();
    	}
    	
    	_hangGearController.setIsChassisControlEnabled(true);

    	// #### Shooter ####
    	_shooter.FullStop();
    	_shooter.MoveActuatorToDefaultPosition();
    	_shooter.ResetHopperCarousel();
    	
    	// #### Ball Infeed ####
    	_ballInfeed.FullStop();
    	
    	_navX.zeroYaw();
    	
    	// #### Cameras ####
    	// set to default camera
    	//_switchableCameraServer.ChgToCamera(RobotMap.BALL_INFEED_CAMERA_NAME);
    	
    	// #### Telop Controller ####
    	_teleopMode = TELEOP_MODE.STANDARD;	// default to std mode
    	
    	_roboRealmClient.ChangeToCamera(VISION_CAMERAS.BOILER);
    	
    	// #### Lidar starts doing ####
    	//if(_lidar != null)	{ _lidar.start(); }	//TODO: resolve timeout
    	
    	//=========================================================
    	//Step 2
    	//Set the Proper Cameras for this match
    	//==========================================================
    	/*_switchableCameraServer.setGearCameraName(_dashboardInputs.get_gearCam().get_cameraName());
    	_switchableCameraServer.setShooterCameraName(_dashboardInputs.get_shooterCam().get_cameraName());
    	_switchableCameraServer.setClimberCameraName(_dashboardInputs.get_climberCam().get_cameraName());
    	_switchableCameraServer.setDriverCameraName(_dashboardInputs.get_driverCam().get_cameraName());*/
    	
    	// =====================================
    	// Step 2.2: Turn all vision LEDs offg
    	// =====================================
    	_roboRealmClient.TurnAllVisionLEDsOff();
    	
    	// =====================================
    	// Step 3: Configure Logging (if USB Memory Stick is present)
    	// =====================================    	
    	_dataLogger = GeneralUtilities.setupLogging("telop");
    	
    	_lastDashboardWriteTimeMSec = new Date().getTime();
	}
	
	// --------------------------------------------------------------------------------------------------------------------------------------------
	// Code executed every scan cycle (about every 20 mSec or 50x / sec) in TELOP Mode							TELOP PERIODIC
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void teleopPeriodic() {
    	// =====================================
    	// Step 0: Get the DASHBOARD Input values for the current scan cycle
    	// =====================================
    	_driversStation.ReadCurrentScanCycleValues();
    	
    	// =====================================
    	// Step 1: execute different steps based on current "telop mode"
    	// =====================================
    	switch (_teleopMode) {
    		case STANDARD:	  
    			//DriverStation.reportError(Double.toString(_navX.getYaw()), false);
    			//===========================================================================
    			//Switchable Cameras
    			//=======================================================================			
    			if(_driversStation.getIsOperator_SwapCamera_BtnJustPressed()) {
    				_switchableCameraServer.ChgToNextCamera();
    			}
    			else if (_driversStation.getIsEngineering_SwapCamera_BtnJustPressed()) {
    				_switchableCameraServer.ChgToNextCamera();
    			}
				//=====================
		    	// Chassis Gear Shift
				//=====================
		    	if(_driversStation.getIsDriver_ShiftDrivingGear_BtnJustPressed()) {
		    		_chassis.ToggleShiftGear();
		    	}

		    	//=====================
		    	// Chassis Throttle Cmd
				//=====================
		    	if ((Math.abs(_driversStation.getDriver_ChassisThrottle_JoystickCmd()) > 0.0) 
		    			|| (Math.abs(_driversStation.getDriver_ChassisTurn_JoystickCmd()) > 0.0)) {
		    		_chassis.EnablePercentVBusMode();
		    		// std drive
			    	_chassis.ArcadeDrive((_driversStation.getDriver_ChassisThrottle_JoystickCmd() * -1.0), 	// added -1 gear is front
											_driversStation.getDriver_ChassisTurn_JoystickCmd());
		    	} 
		    	else if ((Math.abs(_driversStation.getDriver_SpinChassisLeft_JoystickCmd()) > 0.0)
		    				&& (Math.abs(_driversStation.getDriver_SpinChassisRight_JoystickCmd()) == 0.0)) {
		    		// spin left
		    		_chassis.ArcadeDrive(0.0, _driversStation.getDriver_SpinChassisLeft_JoystickCmd() * 0.75 * -1.0);
		    	} 
		    	else if ((Math.abs(_driversStation.getDriver_SpinChassisRight_JoystickCmd()) > 0.0) 
		    				&& (Math.abs(_driversStation.getDriver_SpinChassisLeft_JoystickCmd()) == 0.0)) {
		    		// spin right
		    		_chassis.ArcadeDrive(0.0, _driversStation.getDriver_SpinChassisRight_JoystickCmd() * 0.75);
		    	}
		    	else if ((Math.abs(_driversStation.getEngineering_SpinChassisLeft_JoystickCmd()) > 0.0)
	    				&& (Math.abs(_driversStation.getEngineering_SpinChassisRight_JoystickCmd()) == 0.0)) {
		    		// spin left
		    		_chassis.ArcadeDrive(0.0, _driversStation.getEngineering_SpinChassisLeft_JoystickCmd() * 0.75 * -1.0);
		    	} 
		    	else if ((Math.abs(_driversStation.getEngineering_SpinChassisRight_JoystickCmd()) > 0.0) 
		    				&& (Math.abs(_driversStation.getEngineering_SpinChassisLeft_JoystickCmd()) == 0.0)) {
		    		// spin right
		    		_chassis.ArcadeDrive(0.0, _driversStation.getEngineering_SpinChassisRight_JoystickCmd() * 0.75);
		    	} 
		    	else if (_driversStation.getIsOperator_VisionAim_BtnPressed()
		    				|| _driversStation.getIsEngineering_VisionAim_BtnPressed()) {
		    		// Turn on auto aiming with vision (for boiler)
		    		_chassis.EnableMotionMagicMode();
		    		_chassisAutoAimGyro.motionMagicMoveToTarget(_navX.getYaw() - (_roboRealmClient.get_Angle()/3.5));
		    	} else {
		    		// full stop
		    		// 23.Apr.2017 TomB removed comments
		    		// this avoids ERROR  1  Robot Drive... Output not updated often enough.  java.lang.Thread.run(Thread.java:745) error message
		    		_chassis.EnablePercentVBusMode();
			    	_chassis.ArcadeDrive(0.0, 0.0);
		    	}
		    	
    			//============================================================================
    			// Fuel Infeed Cmd
    			//===========================================================================   			
    			_ballInfeed.InfeedFuelAndExtendSolenoid(_driversStation.getOperator_FuelInfeedOutfeed_JoystickCmd());
    			    			
    			//=====================
    			// Shooter Table
    			//=====================	
    			//if (_driversStation.getIsOperator_IndexShooterSettingsDown_BtnPressed()
    			//	&& _driversStation.getIsOperator_IndexShooterSettingsUp_BtnPressed())
    			if (_driversStation.getIsOperator_AutoDistance_BtnPressed())
    			{
    				_shooter.CalcAutomaticShooter(_roboRealmClient.get_DistanceToBoilerInches());
    			}
    			else if(_driversStation.getIsOperator_IndexShooterSettingsUp_BtnJustPressed()) {
    				_shooter.IndexShooterTableUp();
    			} 
    			else if (_driversStation.getIsOperator_IndexShooterSettingsDown_BtnJustPressed()) {
    				_shooter.IndexShooterTableDown();
    			}
    			
    			//=====================
    			// Shooter Slider (manual control)
    			//=====================		
    			if(_driversStation.getIsOperator_MoveShooterSliderUp_BtnJustPressed()) {
    				_shooter.MoveActuatorUp();
    			} 
    			else if (_driversStation.getIsOperator_MoveShooterSliderDown_BtnJustPressed()) {
    				_shooter.MoveActuatorDown();
    			}
    			else if(_driversStation.getIsEngineering_MoveShooterSliderUp_BtnJustPressed()) {
    				_shooter.MoveActuatorUp();
    			} 
    			else if (_driversStation.getIsEngineering_MoveShooterSliderDown_BtnJustPressed()) {
    				_shooter.MoveActuatorDown();
    			}
    			
    			//=====================
    			// Run Shooter Motors
    			//=====================
				// Stg 1 Bump Up / Down
    			if(_driversStation.getIsEngineering_BumpStg1RPMUp_BtnJustPressed()) {
    				_shooter.BumpStg1MtrRPMUp();
    			}
    			else if (_driversStation.getIsEngineering_BumpStg1RPMDown_BtnJustPressed()) {
    				_shooter.BumpStg1MtrRPMDown();
				}

    			// Stg 2 Bump Up / Down
    			if(_driversStation.getIsEngineering_BumpStg2RPMUp_BtnJustPressed()) {
    				_shooter.BumpStg2MtrRPMUp();
    			}
    			else if (_driversStation.getIsEngineering_BumpStg2RPMDown_BtnJustPressed()) {
    				_shooter.BumpStg2MtrRPMDown();
				}
    			    	
    			//=====================
    			// Toggle Shooter Motors
    			//=====================
    			if(_driversStation.getIsOperator_ToggleShooterMotors_BtnJustPressed()) {
    				_shooter.ToggleShooterMotors();
    			}
    			else if(_driversStation.getIsEngineering_ToggleShooterMotors_BtnJustPressed()) {
    				_shooter.ToggleShooterMotors();
    			}
    			else if (_shooter.get_isShooterMotorsReentrantRunning()) {
    				_shooter.ShooterMotorsReentrant();
    			}
    			 			
    			//=====================
    			// Shooter Feeder (Magic Carpet & High Roller) Motors controlled as a unit
    			//=====================
    			if(_driversStation.getIsOperator_RunShooterFeederInReverse_BtnPressed()){
    				_shooter.RunShooterFeederInReverse();
    			}
    			
    			else if(_driversStation.getOperator_FireBall_BtnPressed()
    					&& !_shooter.get_isShooterInfeedReentrantRunning()) {
    				// start motors on initial press
    				_shooter.ToggleRunShooterFeeder();
    				_shooter.ToggleHopperCarousel();
    			}
    			else if(_driversStation.getEngineering_FireBall_BtnPressed()
    					&& !_shooter.get_isShooterInfeedReentrantRunning()) {
    				// start motors on initial press
    				_shooter.ToggleRunShooterFeeder();
    				_shooter.ToggleHopperCarousel();
    			}
    			
    			else if(_driversStation.getOperator_FireBall_BtnPressed()
    					&& _shooter.get_isShooterInfeedReentrantRunning()) {
    				// keep calling if still pressed
    				_shooter.RunShooterFeederReentrant();
    				_shooter.RunHopperCarousel();
    			}
    			else if(_driversStation.getEngineering_FireBall_BtnPressed()
    					&& _shooter.get_isShooterInfeedReentrantRunning()) {
    				// keep calling if still pressed
    				_shooter.RunShooterFeederReentrant();
    				_shooter.RunHopperCarousel();
    			}
    			
    			else if(!_driversStation.getOperator_FireBall_BtnPressed()
    					&& _shooter.get_isShooterInfeedReentrantRunning()) {
    				// if it was running and is no longer pressed
    				_shooter.ToggleRunShooterFeeder();
    			}
    			else if(!_driversStation.getEngineering_FireBall_BtnPressed()
    					&& _shooter.get_isShooterInfeedReentrantRunning()) {
    				// if it was running and is no longer pressed
    				_shooter.ToggleRunShooterFeeder();
    			} else {
    				// we need to shut off the motors if they were running in reverse and the reverse button was released
    				_shooter.CleanupRunShooterFeederInReverse();
    			}
    			
    					
		    	//=====================
		    	// Gear Tilt Cmd
		    	//	Note: All of the Gear Handler sequences are interruptable except for Zero!
				//=====================
		      	if(!_gearHandler.hasTiltAxisBeenZeroed()) {
		      		// 1st priority is zeroing
		      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
		      		//			we must treat it as a Reentrant function
		      		//			and automatically recall it until complete
		    		_gearHandler.ZeroGearTiltAxisReentrant();
		    	}
		      	else if (_driversStation.getIsDriver_RezeroGearTilt_ButtonJustPressed()) {
		      		 //2nd priority is operator request to rezero
		      		_gearHandler.ZeroGearTiltAxisInit();	// zeroing will start on next scan
		      	}
		      	//else if (Math.abs(_driversStation.getOperator_GearTiltFeed_JoystickCmd()) > 0.0) {
		      		// 3rd priority is joystick control
		      		//_gearHandler.MoveTiltAxisVBus(_driversStation.getOperator_GearTiltFeed_JoystickCmd(), false);
		      	//}
		      	else if (_driversStation.getIsDriver_SendGearTiltToHome_BtnJustPressed()) {
		      		// 4th priority is Goto Home
		      		_gearHandler.MoveGearToHomePosition();
		      		//DriverStation.reportError("NavX: " + Double.toString(_navX.getYaw()), false);
		      		//DriverStation.reportError("Vision: " + Double.toString(_roboRealmClient.get_Angle()), false);
		      	}
		      	else if (_driversStation.getIsDriver_SendGearTiltToScore_BtnJustPressed()) {
		      		// 5th priority is Goto Score
		      		_gearHandler.MoveGearToScorePosition();
		      	} 
		      	else if (_driversStation.getIsDriver_SendGearTiltToFloor_BtnJustPressed()
		      				|| !_gearHandler.getIsLastTiltMoveToFloorCallComplete()) {
		      		// 6th priority is Goto Floor
		      		// 	Note: MoveToFloor will take longer than 1 scan cycle to complete so
		      		//			we must treat it as a Reentrant function
		      		//			and automatically recall it until complete
		      		_gearHandler.MoveGearToFloorPositionReentrant();
		      	} 
		      	//else if(_driversStation.getIsDriver_SendGearTiltToFloor_BtnPressed()
	      		//		&& _gearHandler.getIsLastTiltMoveToFloorCallComplete()) {
	      		//	_gearHandler.MoveTiltAxisVBus(0.1);
	      		//} 
		      	//else if (!_driversStation.getIsDriver_SendGearTiltToFloor_BtnPressed()
	      		//		&& _gearHandler.getIsLastTiltMoveToFloorCallComplete()){
	      		//	_gearHandler.MoveTiltAxisVBus(0.0);
	      		//}
		      	
		    	//=====================
		    	// Gear Infeed/Outfeed Cmd
				//=====================
		      	
		      	if (_driversStation.getIsDriver_InfeedGear_BtnPressed()) {
		      		_gearHandler.SpinInfeedWheelsVBus(1.0);
		      	}
		      	else if (_driversStation.getIsDriver_OutfeedGear_BtnPressed()) {
		      		_gearHandler.SpinInfeedWheelsVBus(-1.0);
		      	}
		      	else {
		      		_gearHandler.SpinInfeedWheelsVBus(0.0);
		      	} 
		      	/*
		      	if (_driversStation.getIsDriver_InfeedGear_BtnJustPressed()) {
		      		_autoShootController.InitializeVisionAiming();
		      	}
		      	if (_driversStation.getIsDriver_InfeedGear_BtnPressed()) {
		      		_autoShootController.AimWithVision();
		      		if (_autoShootController.IsReadyToShoot()) {
		      			DriverStation.reportError("Ready to SHOOT", false);
		      		}
		      	} */
		      	
				//=====================
		    	// ====> Enter Gear Hang Mode
				//=====================
    			if(_driversStation.getIsDriver_StartGearScoreSequence_BtnJustPressed()) {
    				// make sure Gear Tilt has completed zeroing before entering this mode!
    				if(_gearHandler.hasTiltAxisBeenZeroed()) {
	    				_teleopMode = TELEOP_MODE.HANG_GEAR_SEQUENCE_MODE;
	    				_hangGearController.Initialize();
    				} else {
    					DriverStation.reportWarning("=!=!= Cannot chg to Hang Gear Seq, Tilt is NOT finished zeroing yet =!=!=", false);
    				}
    			}	
		      	
    	    	// =====================================
    	    	// ====> Enter Climb Mode
    	    	// =====================================
    			// Original Current Limited Mode Start/Stop using a button
    	    	//if (_driversStation.getIsDriver_StartClimb_ButtonJustPressed()) {		// TODO: put this back in
    	    	//	_teleopMode = TELEOP_MODE.CLIMBING;
    	    	//	_climber.RunClimberReentrant();
    	    	//}
    	    	
    			// 
    	    	//if (_driversStation.getIsDriver_Climb_ButtonPressed()) {
    	    	//	_climber.RunMotorTest(Climber.CLIMBER_MOTOR_VBUS);
    	    	//}
    	    	//else {
    	    	//	_climber.RunMotorTest(0.0);
    	    	//}
    	    	
    			// climb is now a manual mode for Pittsburgh
    			_climber.RunMotorUsingJoyStick(_driversStation.getOperator_ClimberSpeed_JoystickCmd());
    	    	
		      	break;	// end of _telopMode = STANDARD
      		
    		case HANG_GEAR_SEQUENCE_MODE:
    			
    			// in this teleop mode the driver & operator do not have control until
    			// the sequence completes or it times out
    			boolean isStillRunning = _hangGearController.ExecuteRentrant();
    			
    			// if not still running, switch back to std teleop mode
    			//	(ie: give control back to the driver & operator)
    			if(!isStillRunning) {
    				_teleopMode = TELEOP_MODE.STANDARD;
    			}
    			break;	// end of _telopMode = HANG_GEAR_SEQUENCE_MODE
    			
    		case CLIMBING:

    			// climb is now a manual mode for Pittsburgh
    			// arcade drive
		    	//_chassis.ArcadeDrive((_driversStation.getDriver_ChassisThrottle_JoystickCmd() * -1.0), 	// TomB Gear is now front
				//						_driversStation.getDriver_ChassisTurn_JoystickCmd());
		    	
		    	// climber
		    	//if (_driversStation.getIsDriver_StartClimb_ButtonJustPressed()) {	
		    		// cancel climbing
	    		//	_climber.FullStop();
	    		//	_teleopMode = TELEOP_MODE.STANDARD;
		    	//} else {
		    		// keep calling this method
		    	//	_climber.RunClimberReentrant();
		    	//}
    			break;
    			
    	}	// end of switch statement

    	// =====================================
    	// Step N: Finish up 
    	// =====================================
    	
    	// Refresh Dashboard
    	OutputAllToSmartDashboard();
    	
    	// Optionally Log Data
    	WriteLogData();
	}
		
	@Override
	public void testPeriodic()
	{
    	// =====================================
    	// Step 0: Get the DASHBOARD Input values for the current scan cycle
    	// =====================================
    	_driversStation.ReadCurrentScanCycleValues();
    	
    	// Gear Tilt Zero
    	if(_driversStation.getIsDriver_RezeroGearTilt_ButtonJustPressed()) {
    		_roboRealmClient.TurnAllVisionLEDsOff();
    	}

    	// Driving Gear Shift Toggle
    	if(_driversStation.getIsDriver_ShiftDrivingGear_BtnJustPressed()) {
    		_roboRealmClient.TurnGearVisionLEDsOn();
    	}
    	
    	// Gear Tilt Home
    	if(_driversStation.getIsDriver_SendGearTiltToHome_BtnJustPressed()) {
    		_roboRealmClient.TurnBoilerrVisionLEDsOn();
    	}
	}
	
	//--------------------------------------------------------------------------------------------------------------------------------------------------------
	//  Utility / Helper Methods Follow
	//--------------------------------------------------------------------------------------------------------------------------------------------------------
	
    // utility method that calls the outputToSmartDashboard method on all subsystems
    private void OutputAllToSmartDashboard() {
    	// limit spamming
    	
    	long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    	_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
	    	if(_ballInfeed != null)				{ _ballInfeed.OutputToSmartDashboard(); }
	    	
	    	if(_chassis != null) 				{ _chassis.OutputToSmartDashboard(); }
	    	
	    	if(_climber != null)				{ _climber.OutputToSmartDashboard(); }
	    	
	    	if(_driversStation != null)			{ _driversStation.OutputToSmartDashboard(); }
	    	
	    	if(_gearHandler != null)			{ _gearHandler.OutputToSmartDashboard(); }
	    		
	    	if(_lidar != null)					{ _lidar.OutputToSmartDashboard(); }
	    	
	    	if(_navX != null)					{ _navX.OutputToSmartDashboard(); }
	    	
	    	if(_shooter != null)				{ _shooter.OutputToSmartDashboard(); }
	    	
	    	if(_switchableCameraServer != null) { _switchableCameraServer.OutputToSmartDashboard(); }
	    	
	    	if(_roboRealmClient != null) 		{ _roboRealmClient.OutputToSmartDashboard(); }
	    	
	    	//if(_trajController != null)			{ _trajController.OutputToSmartDashboard(); }
	    	
	    	SmartDashboard.putString("Robot Build", _buildMsg);
	    	SmartDashboard.putString("FMS Debug Msg", _fmsDebugMsg);
	    	
	    	BigDecimal movingAvg = _scanTimeSamples.getAverage();
	    	DecimalFormat df = new DecimalFormat("####");
	    	SmartDashboard.putString("Scan Time (2 sec roll avg)", df.format(movingAvg) + " mSec");
	    	
    		// snapshot last time
    		_lastDashboardWriteTimeMSec = new Date().getTime();
    	}
    	
    	// snapshot when this scan ended
    	_lastScanEndTimeInMSec = new Date().getTime();
    }
         
    // this method optionally calls the UpdateLogData on each subsystem and then logs the data
    private void WriteLogData() {    	
    	if(_dataLogger != null) {    	
	    	// create a new, empty logging class
        	LogData logData = new LogData();
	    	
	    	// ask each subsystem that exists to add its data
        	// 23.Apr.2017 TomB Commented most out to improve logging perf
	    	if(_chassis != null) 			{ _chassis.UpdateLogData(logData); }
	    	
	    	//if(_climber != null) 			{ _climber.UpdateLogData(logData); }
	    		
	    	//if(_driversStation != null) 	{ _driversStation.UpdateLogData(logData); }
	    	
	    	//if(_gearHandler != null) 		{ _gearHandler.UpdateLogData(logData); }
	    	
	    	//if(_ballInfeed != null) 		{ _ballInfeed.UpdateLogData(logData); }
	    	
	    	//if(_lidar != null)				{ _lidar.UpdateLogData(logData); }
	    	
	    	//if(_navX != null) 				{ _navX.UpdateLogData(logData); }
	    	
	    	if(_shooter != null)			{ _shooter.UpdateLogData(logData); }
	    	
	    	//if(_roboRealmClient != null) 	{ _roboRealmClient.UpdateLogData(logData); }
	    	
	    	// now write to the log file
	    	_dataLogger.WriteDataLine(logData);
    	}
    }
}