package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.utilities.LogData;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallInfeed extends Subsystem{
	
	CANTalon _fuelInfeedMtr;
	Solenoid _fuelInfeedSolenoid;
	
	public BallInfeed(int fuelInfeedMtrCanBusAddr, int PCMCanAddr, int fuelInfeedSolenoidPort) {
		_fuelInfeedMtr = new CANTalon(fuelInfeedMtrCanBusAddr);
		_fuelInfeedMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
		_fuelInfeedMtr.enableBrakeMode(false);							// default to brake mode DISABLED
		_fuelInfeedMtr.enableLimitSwitch(false, false);					//no limit switches
		
		_fuelInfeedSolenoid = new Solenoid(PCMCanAddr, fuelInfeedSolenoidPort);
	}
	
	public void InfeedFuelAndExtendSolenoid(double percentVBusCmd) {
		// run motor using joystick cmd
		_fuelInfeedMtr.set(percentVBusCmd * -1.0);
		
		// if running fwd or reverse fire solenoid
		if(percentVBusCmd != 0) {
			_fuelInfeedSolenoid.set(true);			//extend Solenoid
		} else {
			_fuelInfeedSolenoid.set(false);			//retract Solenoid
		}
	}
	
	@Override
	public void stop() {
		_fuelInfeedSolenoid.set(false);				//retract Solenoid
		_fuelInfeedMtr.set(0);						//stop motors	
	}
	
	@Override
	public void zeroSensors() {
	}
		
	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("Is Fuel Infeed Tilt Extended", _fuelInfeedSolenoid.get());
		
		String ballInfeedMtrData = "?";
		
		if(Math.abs(_fuelInfeedMtr.getOutputVoltage()) > 0) {
			ballInfeedMtrData = String.format("%s (%.0f%%)", 
												"ON", 
												(_fuelInfeedMtr.getOutputVoltage() / _fuelInfeedMtr.getBusVoltage())* 100);
		} else {
			ballInfeedMtrData = String.format("%s (%.0f%%)", "off", 0.0);
		}
		
		SmartDashboard.putString("Fuel Infeed", ballInfeedMtrData);
	}
	
	@Override
	public void updateLogData(LogData logData) {
	}
}