/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveClass.SwerveDrive;
import frc.robot.SwerveClass.SwervePOD;
import frc.robot.SwerveClass.Utils;

import com.ctre.*;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static SwervePOD[] swervePOD  = new SwervePOD[4];
  public static SwerveDrive swerveDrive;
  protected double kP = 0.005;
	protected double kI = 0.00;
	protected double kD = 0.005;
  private int deadBand = 1; //
  private Joystick joy = new Joystick(0);
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    swervePOD[0] = new SwervePOD(0, 1, 0);
    swervePOD[1] = new SwervePOD(2, 3, 1);
    swervePOD[2] = new SwervePOD(4, 5, 2);
    swervePOD[3] = new SwervePOD(6, 7, 3);

    for(int i=0;i<4;i++){
      swervePOD[i].initialize();
      swervePOD[i].configPIDTurn(kP, kI, kD, deadBand, 1.0);
    }
    swerveDrive = new SwerveDrive(swervePOD[0], swervePOD[0], swervePOD[0], swervePOD[0], 10);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  
  
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double x = Utils.limitMagnitude(Utils.joyExpo(joy.getX(), 1.5), 0.05, 1.0);
		double y = Utils.limitMagnitude(Utils.joyExpo(joy.getY(), 1.5), 0.05, 1.0);
		double z = Utils.limitMagnitude(Utils.joyExpo(joy.getRawAxis(4), 1.5), 0.05, 0.5);
    //use gyro for field orientation
		//swerveDrive.drive(x, y, z, gyroAngle);

		swerveDrive.drive(x, y, z, 0);

  }


}
