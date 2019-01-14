package frc.robot.SwerveClass;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;

public class Team801TalonSRX extends TalonSRX implements PIDOutput
{

	public Team801TalonSRX(int deviceNumber) {
		super(deviceNumber);
	}

	@Override
	public void pidWrite(double output)
	{
		this.set(getControlMode(), output);
	}
	
}
