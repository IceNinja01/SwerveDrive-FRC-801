package frc.robot.SwerveClass;


public class Utils
{

	public static double wrapAngle0To360Deg(double angle_deg) {
		// keep heading command between 0 and 360
		angle_deg = angle_deg % 360.0;  
		if (angle_deg < 0) angle_deg += 360.0;
		return angle_deg;
	

	}
	
	public static double convertUnitDegtoHeadingDeg(double angle_deg) {
		if(angle_deg > 0 && angle_deg < 90) {
			angle_deg -= 90;
			angle_deg *= -1;
		}
		if(angle_deg > 90 && angle_deg < 180) {
			angle_deg -= 180;
			angle_deg *= -1;
			angle_deg += 270;
		}
		if(angle_deg > 180 && angle_deg < 270) {
			angle_deg -= 270;
			angle_deg *= -1;
			angle_deg += 180;
		}
		if(angle_deg > 270 && angle_deg < 360) {
			angle_deg -= 360;
			angle_deg *= -1;
			angle_deg += 90;
		}
		
		
		return angle_deg;
	}
	
	public static double convertDegtoRad(double angle_deg) {
		/*Command is used to convert from degrees to radians
		 * First : degrees have to be converted to unit circle degrees instead of heading degrees
		 Then use toRadians
		 */
		if(angle_deg > 0 && angle_deg < 90) {
			angle_deg -= 90;
			angle_deg *= -1;
		}
		if(angle_deg > 90 && angle_deg < 180) {
			angle_deg -= 180;
			angle_deg *= -1;
			angle_deg += 270;
		}
		if(angle_deg > 180 && angle_deg < 270) {
			angle_deg -= 270;
			angle_deg *= -1;
			angle_deg += 180;
		}
		if(angle_deg > 270 && angle_deg < 360) {
			angle_deg -= 360;
			angle_deg *= -1;
			angle_deg += 90;
		}
		angle_deg = Math.toRadians(angle_deg);
		return angle_deg;
		
	}
	
	public static double limitMagnitude(double in , double min, double max)
	{
		double out = Math.abs( in ); //Remove sign for bounds testing
		if(out > max)
		{
			out = max;
		}
		if(out < min)
		{
			out = 0.0;
		}
		return out * Math.signum( in ); //Re-add sign
	}
	
	public static double limit(double in , double min, double max)
	{
		double out = in;
		if(out > max)
		{
			out = max;
		}
		if(out < min)
		{
			out = min;
		}
		return out;
	}
	
	public static double dead(double in , double deadband)
	{
		return Math.abs(in) > deadband ? in : 0;
	}
	
	public static boolean between(double in, double min, double max)
	{
		return (min < in && in < max)?true:false;
	}

	public static double joyExpo(double stickInput,double exp)
	{
		//  This function takes a joystick input and applies an exponential scaling
		//  For expo=1.5, the return value varies from about 50% of commanded at low inputs,
		//  To 80-100% of commanded at high rates
		double stickOutput;
		
		//Convert linear input magnitude to exponential from 0
		stickOutput = Math.exp(Math.abs(stickInput) * exp) -1;
		//Normalize to unit magnitude
		stickOutput = stickOutput / (Math.exp(exp) - 1);
		//Reapply polarity
		stickOutput = Math.signum(stickInput) * stickOutput;
		
		return stickOutput;
	}
}