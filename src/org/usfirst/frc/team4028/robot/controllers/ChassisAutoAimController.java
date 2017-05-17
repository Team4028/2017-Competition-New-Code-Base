package org.usfirst.frc.team4028.robot.controllers;

import org.usfirst.frc.team4028.robot.util.BeefyMath;
import org.usfirst.frc.team4028.robot.util.PIDCalculator;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.DriverStation;

public class ChassisAutoAimController {
	PIDCalculator _autoAimPID;
	Chassis _chassis;
	NavXGyro _navX;
	
	public ChassisAutoAimController(Chassis chassis, NavXGyro navX, double p, double i, double d) {
		_chassis = chassis;
		_navX = navX;
		_autoAimPID = new PIDCalculator(p, i, d);
	}
	
	// =========================================================
	// Methods
	// =========================================================
	public boolean onTarget() {
		return _autoAimPID.onTarget(); // Check if the PID loop error is within a set deadband
	}
	
	public void loadNewTarget(double angle) {
		_autoAimPID.reset();			// Reset the PID Calculator
		_autoAimPID.setSetpoint(angle); // Set a new target angle
		DriverStation.reportError("New Setpoint Loaded", false);
	}
	
	public void loadNewVisionTarget(double angle) {
		_autoAimPID.setSetpoint(_navX.getYaw() - angle);
	}
	
	public void moveToTarget() {
		double motorOutput = _autoAimPID.calculate(_navX.getYaw()); // Pass in current angle to calculate motor output
		if (motorOutput == 0.0) {
			_chassis.EnableBrakeMode(true);
		} else {
			_chassis.EnableBrakeMode(false);
		}
		_chassis.TankDrive(-motorOutput, motorOutput);
	}
	
	public void motionMagicMoveToTarget(double target) {
		_chassis.EnableMotionMagicMode();
		
		double angleError = target - _navX.getYaw();
		
		double encoderError = BeefyMath.degreesToEncoderRotations(angleError);
		
		double leftDriveTargetPosition = _chassis.getLeftEncoderCurrentPosition() - encoderError;
		double rightDriveTargetPosition = _chassis.getRightEncoderCurrentPosition() + encoderError;
		
		_chassis.SetMotionMagicTargetPosition(leftDriveTargetPosition, rightDriveTargetPosition);
	}
	
	public void zeroTotalError() {
		_autoAimPID.resetTotalError();
	}
	
	public void setDeadband(double deadband) {
		_autoAimPID.setDeadband(deadband);
	}
	
	public void setMaxMinOutput(double max, double min) {
		_autoAimPID.setOutputRange(min, max);
	}
	
	public double currentHeading() {
		return _navX.getYaw();
	}
	
	public void ChassisFullStop() {
		_chassis.FullStop();
	}
}