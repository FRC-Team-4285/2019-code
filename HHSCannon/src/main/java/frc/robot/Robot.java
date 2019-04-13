/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {

  Joystick stick = new Joystick(0);


  Talon left_drive_motor = new Talon(4);
  Talon right_drive_motor = new Talon(2);
  Talon vertical_shooter_motor = new Talon(1);
  Talon horizontal_shooter_motor = new Talon(0);

  DoubleSolenoid confetti1 = new DoubleSolenoid(0, 1);
  DoubleSolenoid confetti2 = new DoubleSolenoid(2, 3);
  DoubleSolenoid confetti3 = new DoubleSolenoid(4, 5);
  DoubleSolenoid shooter = new DoubleSolenoid(6, 7);
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    left_drive_motor.setInverted(true);
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
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    boolean bconfetti2 = stick.getRawButtonPressed(5);
    boolean sconfetti2 = stick.getRawButtonReleased(5);
    boolean bconfetti3 = stick.getRawButtonPressed(6);
    boolean sconfetti3 = stick.getRawButtonReleased(6);

    boolean up_vertical = stick.getRawButtonPressed(4);
    boolean up_vertical_stop = stick.getRawButtonReleased(4);
    boolean down_vertical = stick.getRawButtonPressed(1);
    boolean down_vertical_stop = stick.getRawButtonReleased(1);

    boolean right_horizontal = stick.getRawButtonPressed(2);
    boolean right_horizontal_stop = stick.getRawButtonReleased(2);
    boolean left_horizontal = stick.getRawButtonPressed(3);
    boolean left_horizontal_stop = stick.getRawButtonReleased(3);
    
    if (stick.getRawAxis(1) > 0.1 || stick.getRawAxis(1) < -0.1) 
    {
      left_drive_motor.set(stick.getRawAxis(1)/2);
    }

    if (stick.getRawAxis(1) < 0.1 && stick.getRawAxis(1) > -0.1)
    {
      left_drive_motor.set(0);
    }

    if (stick.getRawAxis(5) > 0.1 || stick.getRawAxis(5) < -0.1) 
    {
      right_drive_motor.set(stick.getRawAxis(5));
    }

    if (stick.getRawAxis(5) < 0.1 && stick.getRawAxis(5) > -0.1)
    {
      right_drive_motor.set(0);
    }
    
    if (stick.getRawAxis(3) > 0.4)
    {
      shooter.set(DoubleSolenoid.Value.kReverse);
    }
    if (stick.getRawAxis(3) < 0.4)
    {
      shooter.set(DoubleSolenoid.Value.kForward);
    }

    if (stick.getRawAxis(2) > 0.4)
    {
      confetti1.set(DoubleSolenoid.Value.kReverse);
    }
    if (stick.getRawAxis(2) < 0.4)
    {
      confetti1.set(DoubleSolenoid.Value.kReverse);
    }
    
    if (up_vertical)
    {
      vertical_shooter_motor.set(.2);
    }
    if (up_vertical_stop)
    {
      vertical_shooter_motor.set(0);
    }

    if (down_vertical)
    {
      vertical_shooter_motor.set(-.2);
    }
    if (down_vertical_stop)
    {
      vertical_shooter_motor.set(0);
    }

    if (left_horizontal)
    {
      horizontal_shooter_motor.set(.2);
    }
    if (left_horizontal_stop)
    {
      horizontal_shooter_motor.set(0);
    }

    if (right_horizontal)
    {
      horizontal_shooter_motor.set(-.2);
    }
    if (right_horizontal_stop)
    {
      horizontal_shooter_motor.set(0);
    }
    
    if (bconfetti2)
    {
      confetti2.set(DoubleSolenoid.Value.kReverse);
    }
    if (sconfetti2)
    {
      confetti2.set(DoubleSolenoid.Value.kForward);
    }

    if (bconfetti3)
    {
      confetti3.set(DoubleSolenoid.Value.kReverse);
    }
    if (sconfetti3)
    {
      confetti3.set(DoubleSolenoid.Value.kForward);
    }
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
