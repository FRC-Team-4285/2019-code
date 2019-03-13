/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {

  Joystick stick = new Joystick(0);
	
	TalonSRX Motor1 = new TalonSRX(1);
	TalonSRX Motor6 = new TalonSRX(6);
	TalonSRX Motor8 = new TalonSRX(8);
	TalonSRX Motor9 = new TalonSRX(9);
  
  TalonSRX Motor12 = new TalonSRX(12);

  Timer RobotTimer = new Timer();  
  
  Encoder encoder0 = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
  Encoder encoder1 = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
  Encoder encoder2 = new Encoder(4, 5, false, Encoder.EncodingType.k2X);
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    Motor8.setInverted(true);
    Motor9.setInverted(true);

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
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    
  
    System.out.println(RobotTimer.get());
		
		if(RobotTimer.get() < 2) {
		Motor1.set(ControlMode.PercentOutput, -0.2);
		Motor6.set(ControlMode.PercentOutput, -0.2);
		Motor8.set(ControlMode.PercentOutput, -0.2);
		Motor9.set(ControlMode.PercentOutput, -0.2);
		}
		else {
			Motor1.set(ControlMode.PercentOutput, 0);
			Motor6.set(ControlMode.PercentOutput, 0);
			Motor8.set(ControlMode.PercentOutput, 0);
			Motor9.set(ControlMode.PercentOutput, 0);
		}
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    System.out.println(encoder2.get());
    
    encoder2.setDistancePerPulse(0.1);
    int count = encoder2.get();

    boolean Eup = stick.getRawButton(4);
    boolean Edown = stick.getRawButton(1);
    boolean Estop = stick.getRawButton(2);
    boolean EDown = stick.getRawButton(3);

    //Motor1.set(ControlMode.PercentOutput, stick.getRawAxis(1));
		//Motor6.set(ControlMode.PercentOutput, stick.getRawAxis(1));
		//Motor8.set(ControlMode.PercentOutput, stick.getRawAxis(5));
    //Motor9.set(ControlMode.PercentOutput, stick.getRawAxis(5));

    if(Eup){
      if(encoder2.get() > 1010){
        Motor12.set(ControlMode.PercentOutput, -0.5);
      }
      else if (encoder2.get() < 990){
        Motor12.set(ControlMode.PercentOutput, 0.5);
      }
      else{
        Motor12.set(ControlMode.PercentOutput, 0);
      }
    }
		
    if(Edown) {
      if(encoder2.get() > 510){
        Motor12.set(ControlMode.PercentOutput, -0.5);
      }
      else if (encoder2.get() < 490){
        Motor12.set(ControlMode.PercentOutput, 0.5);
      }
      else{
        Motor12.set(ControlMode.PercentOutput, 0);
      }
    }

    if(Estop) {
      if(encoder2.get() > 10){
        Motor12.set(ControlMode.PercentOutput, -0.5);
      }
      else if (encoder2.get() < -10){
        Motor12.set(ControlMode.PercentOutput, 0.5);
      }
      else{
        Motor12.set(ControlMode.PercentOutput, 0);
      }
    }
    if(EDown){
      Motor12.set(ControlMode.PercentOutput, -0.5);
    }
}


  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

