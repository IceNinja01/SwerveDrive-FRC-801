package frc.robot.SwerveClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.graalvm.compiler.core.common.type.ArithmeticOpTable.UnaryOp.Sqrt;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This code use Ether's calculations to perform the inverse kinamatics equations for swerve drive
//https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383



@SuppressWarnings("unused")
public class SwerveDrive implements MotorSafety {


	//Variables to be used are set to private
	
	protected MotorSafetyHelper m_safetyHelper;
	protected PIDOutput[] pidOutput = new PIDOutput[4]; 
	private PIDController[] pidTurnController  = new PIDController[4];
	private PIDSource[] pidTurnSource  = new PIDSource[4];
	public static final double kDefaultExpirationTime = 0.1;
	public static final double kDefaultMaxOutput = 1.0;

	protected double temp, STR, FWD, RCW;
	protected double A,B,C,D, max;
	protected double L = 33.5; //Lenght of robot wheel base
	protected double W = 28.25; //Width of robot wheel base
	protected double R = Math.sqrt(L*L+W*W);
	protected double kP = 0.005;
	protected double kI = 0.00;
	protected double kD = 0.005;
	protected double timeUs;
	private String motorName[] = {"FrontRight","FrontLeft","BackLeft","BackRight"};

	private double[] oldAngle = {0,0,0,0};
	private double maxDrive = 1.0;
	private double maxTurn = 1.0;

	private int deadBand = 1; //
	private SwervePOD[] SwervePOD  = new SwervePOD[4];
    private double[] wheelAngles = new double[4];
	private double[] wheelSpeeds = new double[4];
	private double[] angleJoyStickDiff = new double[4];
	private double[] angleError = new double[4];

	private RollingAverage xavg;
	private RollingAverage yavg;
	private RollingAverage zavg;	
	
	public  SwerveDrive(SwervePOD FrontRightPOD, SwervePOD FrontLeftPOD, SwervePOD BackLeftPOD,SwervePOD BackRightPOD,
			int avgSize) {
		
		SwervePOD[0] = FrontRightPOD;
		SwervePOD[1]  = FrontLeftPOD;
		SwervePOD[2]   = BackLeftPOD;
		SwervePOD[3]  = BackRightPOD;
		/*Set the Current Limit of the Drive Motors	 */
		setDriveCurrentLimit(10, 200, 20);
		
		// Initializes the _avg variables to size avgSize
		xavg = new RollingAverage(avgSize);
		yavg = new RollingAverage(avgSize);
		zavg = new RollingAverage(avgSize);
		
	}
	 /**
	   * Drive method for Swerve wheeled robots.
	   *	  
	   *
	   * <p>This is designed to be directly driven by joystick axes.
	   *
	   * @param AxisX         	The speed that the robot should drive in the X direction. [-1.0..1.0]
	   * @param AxisY         	The speed that the robot should drive in the Y direction. This input is
	   *                  		inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
	   * @param rotation  		The rate of rotation for the robot that is completely independent of the
	   *                  		translation. [-1.0..1.0]
	   * @param gyroAngle 		The current angle reading from the gyro. Use this to implement field-oriented
	   *                  		controls.
	   */
	public void drive(double AxisX, double AxisY, double rotation, double gyroAngle){
		xavg.add(AxisX);
		yavg.add(AxisY);
		zavg.add(rotation);
		//Calculate Angles and Magnitudes for each motor
		FWD = -yavg.getAverage();
		STR = xavg.getAverage();
		RCW = zavg.getAverage();
		double radians = gyroAngle*Math.PI/180.00;
		temp = FWD*Math.cos(radians) + STR*Math.sin(radians);
		STR = -FWD*Math.sin(radians) + STR*Math.cos(radians);
		FWD = temp;
		//Perform the following calculations for each new set of FWD, STR, and RCW commands:
		A = STR - RCW*(L/R);
		B = STR + RCW*(L/R);
		C = FWD - RCW*(W/R);
		D = FWD + RCW*(W/R);
		
	    wheelSpeeds[0] = Math.sqrt(B*B + C*C);
	    wheelSpeeds[1] = Math.sqrt(B*B + D*D);
	    wheelSpeeds[2] = Math.sqrt(A*A + D*D);
	    wheelSpeeds[3] = Math.sqrt(A*A + C*C);
	    
	    wheelAngles[0] = Utils.wrapAngle0To360Deg(Math.atan2(B,C)*180/Math.PI);
	    wheelAngles[1] = Utils.wrapAngle0To360Deg(Math.atan2(B,D)*180/Math.PI);
	    wheelAngles[2] = Utils.wrapAngle0To360Deg(Math.atan2(A,D)*180/Math.PI);
	    wheelAngles[3] = Utils.wrapAngle0To360Deg(Math.atan2(A,C)*180/Math.PI);
		
	    //Normalize wheelSpeeds
	    //determine max motor speed
	    max=wheelSpeeds[0]; 
	    if(wheelSpeeds[1]>max){
	    	max=wheelSpeeds[1]; 
	    }
	    if(wheelSpeeds[2]>max){
	    	max=wheelSpeeds[2]; 
	    }
	    if(wheelSpeeds[3]>max){
	    	max=wheelSpeeds[3];
	    }
	    //Divide by max motor speeds
	    if(max>1){
	    	wheelSpeeds[0]/=max; 
	    	wheelSpeeds[1]/=max; 
	    	wheelSpeeds[2]/=max; 
	    	wheelSpeeds[3]/=max;
	    }

			double[] degs = new double[4];
		    for(int i=0;i<4;i++){
		    	degs[i] = SwervePOD[i].getAngle();
		    	
		    	angleJoyStickDiff[i]= wheelAngles[i]- oldAngle[i];
		    	angleError[i] = wheelAngles[i] - degs[i];

			    if(Math.abs(angleJoyStickDiff[i]) > 90){ //new angle is greater than a 90degree turn, so find shortest path
			    	//reverse translational motors 
			    	SwervePOD[i].drive(maxDrive*wheelSpeeds[i]*4800*4096/600);
			    	
			    	//find new angle
			    	wheelAngles[i] -= 180.0; //subtract 180 degrees
			    	if(wheelAngles[i] < 0){ //wrap to new angle between 0-360
			    		wheelAngles[i] += 360.0;
			    	}
			    	//now the angle is set to move to the shortest path, which is just 180 degrees 
			    	//from the current heading
			    	
			    }    
			    
			    else
			    {
			    	SwervePOD[i].drive(-maxDrive*wheelSpeeds[i]*4800*4096/600);
			    }
				//Turn Motors
			    if(wheelSpeeds[i]>0.1){
			    	pidTurnController[i].setSetpoint(wheelAngles[i]);
			    	oldAngle[i] = wheelAngles[i];
			    }
		    
		    
	    }
		getspeed();
	    	SmartDashboard.putNumber("Angle", angleJoyStickDiff[0]);

		if (m_safetyHelper != null) {
			m_safetyHelper.feed();
      	}

	}
	// Get the speed of the robot and angle from motor values. The angle is calculated from motors.
	// Use gyro for true angle measurements.
	private void getspeed() {
		double vel_X = 0;
		double vel_Y = 0;
		double velocity = 0;
		for(int i=0;i<4;i++){
			vel_X += SwervePOD[i].getSpeed()*Math.cos(SwervePOD[i].getAngle());
			vel_Y +=  SwervePOD[i].getSpeed()*Math.sin(SwervePOD[i].getAngle());
		}
		vel_X /= 4;
		vel_Y /= 4;
		double omega = Utils.wrapAngle0To360Deg(Math.atan2(vel_Y,vel_X)*180/Math.PI);
		velocity = Math.sqrt(vel_Y*vel_Y + vel_X*vel_X);
		SmartDashboard.putNumber("Velocity", velocity);
		SmartDashboard.putNumber("Angle_Omega", omega);

	}

	public void turnMotors(double angle_CMD) {
	    for(int i=0;i<4;i++){
	    	pidTurnController[i].setSetpoint(angle_CMD);
	    }
	}
	
	public void turnMotorsDrive(double angle_CMD , double speed){
	    for(int i=0;i<4;i++){
	    	pidTurnController[i].setSetpoint(angle_CMD);
	    	SwervePOD[i].set(ControlMode.Velocity, -maxDrive*speed*4800*4096/600);
	    }
		
	}
	
	@Override
	public void setExpiration(double timeout) {
		m_safetyHelper.setExpiration(timeout);
	}

	@Override
	public double getExpiration() {
		return m_safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		return m_safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		for(int i=0;i>4;i++){
		    if (SwervePOD[i] != null) {
		      SwervePOD[i].set(ControlMode.Velocity, 0);
		    }
		    if (turnMotors[i] != null) {
		      pidTurnController[i].disable();
		      turnMotors[i].set(ControlMode.PercentOutput, 0.0);

		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}
	public void brakeOn() {
		for(int i=0;i>4;i++){
		    if (SwervePOD[i] != null) {
			  SwervePOD[i].setNeutralMode(NeutralMode.Brake);
		      SwervePOD[i].neutralOutput();
		    }
		    if (turnMotors[i] != null) {
			  pidTurnController[i].disable();

		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}
	public void brakeOff() {
		for(int i=0;i>4;i++){
		    if (SwervePOD[i] != null) {
		    	  SwervePOD[i].setNeutralMode(NeutralMode.Coast);
			      SwervePOD[i].set(ControlMode.Velocity, 0);
			    }
			    if (turnMotors[i] != null) {
				  pidTurnController[i].enable();

		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		m_safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
		return m_safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		return "Swerve Drive";
	}
	
//  @SuppressWarnings("unused")
  private void setupMotorSafety() {
	    m_safetyHelper = new MotorSafetyHelper(this);
	    m_safetyHelper.setExpiration(kDefaultExpirationTime);
	    m_safetyHelper.setSafetyEnabled(true);
	  }
	
	public void getAmps() {
		for(int i=0 ; i<4; i++) {
		   SmartDashboard.putNumber("DriveAmps_"+motorName[i], SwervePOD[i].getAmps);
		   SmartDashboard.putNumber("DriveVolts_"+motorName[i], S);
		}
	}
	
	public double currentSpeed(SwervePODSwervePOD motor, int num){
		double speed = motor.getSelectedSensorVelocity(0);
		SmartDashboard.putNumber("Speed"+motorName[num], speed);
		return speed;
	}

	public void setMaxDrive(double set){
		this.maxDrive = set;
	}
	
	public double getLR() {
		return L/R;
	}
	
	public double getWR() {
		return W/R;
	}

	public void setWidth(double width){
		this.W = width;
	}

	public void setLength(double length){
		this.L = length;
	}
	
	public void setDriveCurrentLimit(int peakAmps, int durationMs, int continousAmps) {
	/* Peak Current and Duration must be exceeded before current limit is activated.
	When activated, current will be limited to Continuous Current.
	Set Peak Current params to 0 if desired behavior is to immediately current-limit. */
		for(int i=0;i>4;i++){
			SwervePOD[i].configPeakCurrentLimit(peakAmps, 10); /* 35 A */
			SwervePOD[i].configPeakCurrentDuration(durationMs, 10); /* 200ms */
			SwervePOD[i].configContinuousCurrentLimit(continousAmps, 10); /* 30A */
			SwervePOD[i].enableCurrentLimit(true); /* turn it on */
		}
	}
	
	
}
