package frc.robot.SwerveClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwervePOD {
	
	private Team801TalonSRX driveMotor;
	private Team801TalonSRX turnMotor;
	private PIDSource pidTurnSource;
	private PIDController pidTurnController;
	private MotorName motorName;

	/**
	 * 
	 * @param Drive Motor Number on PDB for the Drive motor on SwervePOD
	 * @param Turn	Motor Number on PDB for the Turn motor on SwervePOD
	 * @param PODName EnumType for POD Name: RightFront(0), LeftFront(1), LeftBack(2), RightBack(3).
	 */
	
	public SwervePOD(int Drive, int Turn, int PODName) {
//		Initialize motors
		driveMotor  = new Team801TalonSRX(Drive);
		turnMotor  = new Team801TalonSRX(Turn);
		motorName.value= PODName;
	}
		
	public enum MotorName{
		RightFront(0),
		LeftFront(1),
		LeftBack(2),
		RightBack(3);
		public int value;
		MotorName(int initValue){
			
			this.value = initValue;
			
		}		
	}
	
	public void initialize() {
		turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		/* set the peak and nominal outputs, 12V means full */
		turnMotor.configNominalOutputForward(0,10);
		turnMotor.configNominalOutputReverse(0, 10);
		turnMotor.configPeakOutputForward(11.0, 10);
		turnMotor.configPeakOutputReverse(-11.0, 10);
		turnMotor.enableVoltageCompensation(true); 
		/* 0.001 represents 0.1% - default value is 0.04 or 4% */
		turnMotor.configNeutralDeadband(0.001, 10);
		
		//set coast mode
		turnMotor.setNeutralMode(NeutralMode.Coast);
//		//set Voltage for turn motors
		turnMotor.set(ControlMode.PercentOutput, 0.0);
	
	/*the sensor and motor must be
	in-phase. This means that the sensor position must move in a positive direction as the motor
	controller drives positive motor output. To test this, first drive the motor manually (using
	gamepad axis for example). Watch the sensor position either in the roboRIO Web-based
	Configuration Self-Test, or by calling GetSelectedSensorPosition() and printing it to console.
	If the Sensor Position moves in a negative direction while Talon SRX motor output is positive
	(blinking green), then use the setSensorPhase() routine/VI to multiply the sensor position by (-
	1). Then retest to confirm Sensor Position moves in a positive direction with positive motor
	drive.**/
	turnMotor.setSensorPhase(false); 


	driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	/* set the peak and nominal outputs, 12V means full */
	driveMotor.configNominalOutputForward(0, 10);
	driveMotor.configNominalOutputReverse(0, 10);
	driveMotor.configPeakOutputForward(11.0, 10);
	driveMotor.configPeakOutputReverse(-11.0, 10);
	driveMotor.enableVoltageCompensation(true); 
	driveMotor.configNeutralDeadband(0.01, 10);
	driveMotor.configAllowableClosedloopError(0, 0, 10);		
	/* Set the motors PIDF constants**/
	//index 0
	driveMotor.config_kF(0, .026, 10);
	driveMotor.config_kP(0, .051, 10);
	driveMotor.config_kI(0, 0.0, 10);
	driveMotor.config_kD(0, 0.5, 10);
	driveMotor.setSelectedSensorPosition(0, 0, 10);
	}
	
	public void configPIDTurn(double kP, double kI, double kD, double deadBand, double maxTurnVoltage) {
	pidTurnSource = new PIDSource() {				
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {				
		}
		@Override
		public double pidGet() {
			return getAngle();
		}				
		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
	};
	
	pidTurnController = new PIDController(kP, kI, kD, pidTurnSource, turnMotor);
	pidTurnController.setAbsoluteTolerance(deadBand);
	pidTurnController.setInputRange(0, 360);
	pidTurnController.setContinuous(true);
	pidTurnController.setOutputRange(-maxTurnVoltage, maxTurnVoltage);
	pidTurnController.enable();
	
	}
	/**
	 * @parameter phase	used to change the phase of the talon sensor
	*/
	public void configMotionMagic(int velocity, int accel, boolean phase) {
	//set motion magic config
		driveMotor.configMotionCruiseVelocity(velocity, 10);
		driveMotor.configMotionAcceleration(accel, 10);
		driveMotor.configNeutralDeadband(0.001, 10);
		//set coast mode
		driveMotor.setNeutralMode(NeutralMode.Coast);
		//set Velocity Mode for drive motors
		driveMotor.set(ControlMode.Velocity, 0.0);
		driveMotor.setSensorPhase(phase); 
	}
	
	public double getAngle() {
		   int motorNumber = turnMotor.getDeviceID();
		   // Convert timeUs Pulse to angle
		   /* get the period in us, rise-to-rise in microseconds */
		   double timeUs = turnMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
		   // Convert timeUs Pulse to angle	   
		   double degrees = turnMotor.getSensorCollection().getPulseWidthRiseToFallUs()*(360.0/timeUs);
		   SmartDashboard.putNumber("RawAngle_"+motorName.name(), degrees);
		   degrees = Utils.wrapAngle0To360Deg(degrees) - Constants.AngleBias[motorName.value];
		   degrees = Utils.wrapAngle0To360Deg(degrees);
		   SmartDashboard.putNumber(motorName.name()+" turn", degrees);
		   return degrees;
	}
	
	public void drive(double speed) {
		driveMotor.set(ControlMode.Velocity, speed*4800*4096/600);
	}
	
	public void turn(double angle) {
		pidTurnController.setSetpoint(angle);
	}
	
	public void stop() {
		driveMotor.set(ControlMode.Velocity, 0);
		pidTurnController.disable();
		turnMotor.set(ControlMode.PercentOutput, 0.0);
	}
	
	public void brakeOn() {
		driveMotor.setNeutralMode(NeutralMode.Brake);
		driveMotor.neutralOutput();
	  	pidTurnController.disable();

	}
	
	public void brakeOff() {
		driveMotor.setNeutralMode(NeutralMode.Coast);
		driveMotor.set(ControlMode.Velocity, 0);
	  	pidTurnController.enable();
	}
	
	public double getAmps() {
		return driveMotor.getOutputCurrent();
	}
	
	public double getVoltage() {
		return driveMotor.getMotorOutputVoltage();
	}
	
	public void setDriveCurrentLimit(int peakAmps, int durationMs, int continousAmps) {
		driveMotor.configPeakCurrentLimit(peakAmps, 10); /* 35 A */
		driveMotor.configPeakCurrentDuration(durationMs, 10); /* 200ms */
		driveMotor.configContinuousCurrentLimit(continousAmps, 10); /* 30A */
		driveMotor.enableCurrentLimit(true); /* turn it on */
	}

	public double getSpeed(){
		//Add the following constants to make proper speed calculations
		//(kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio)
		return driveMotor.getSelectedSensorVelocity(); 
	}
}
