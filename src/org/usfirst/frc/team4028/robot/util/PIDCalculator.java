package org.usfirst.frc.team4028.robot.util;

public class PIDCalculator {
	private double _p; // "proportional" term
	private double _i; // "integral" term
	private double _d; // "derivative" term
	private double _maximumOutput = 0.7; 
	private double _minimumOutput = -0.7;
	private double _prevError = 0.0;
	private double _totalError = 0.0;
	private double _totalErrorCeiling = 30.0;
	private double _setpoint = 0.0;
	private double _error = 0.0;
	private double _result = 0.0;
	private double _deadband = 5.0;
	
	public PIDCalculator(double Kp, double Ki, double Kd) {
		_p = Kp;
		_i = Ki;
		_d = Kd;
	}
	
	public double calculate (double input) {
		_error = _setpoint - input;		
		if (Math.abs(_error) < 6.0) {
			if (Math.abs(_totalError + _error) < _totalErrorCeiling) {
				_totalError += _error; // Accumulate error when it is under 6 degrees 
			}
		} else {
			_totalError = 0.0;
		}
		
		double proportionalError = Math.abs(_error) < _deadband ? 0 : _error; // output is zero when error is below deadband
		
		_result = (_p * proportionalError + _i * _totalError + _d * (_error - _prevError));
		
		if (_result > _maximumOutput) { // Ensure result is between min and max output
            _result = _maximumOutput;
        } else if (_result < _minimumOutput) {
            _result = _minimumOutput;
        }
        return _result;  
	}
	
	public void setDeadband (double deadband) {
		_deadband = deadband;
	}
	
	public void setOutputRange(double minimumOutput, double maximumOutput) {
		_minimumOutput = minimumOutput;
		_maximumOutput = maximumOutput;
	}
	
	public void setSetpoint(double setpoint) {
        _setpoint = setpoint;
    }
	
	public boolean onTarget() {
        return (Math.abs(_error) < _deadband);
    }
	
    public void reset() {   // Reset all internal terms.
        _prevError = 0;
        _totalError = 0;
        _result = 0;
        _setpoint = 0;
    }
    
    public void resetTotalError() {
    	_totalError = 0;
    }
}