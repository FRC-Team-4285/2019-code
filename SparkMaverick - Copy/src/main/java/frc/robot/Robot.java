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
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

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
private static final boolean Climberin = false;
private static final boolean Climberout = false;
private static final boolean Climberinout = false;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  Joystick stick = new Joystick(0);
  Joystick stick2 = new Joystick(2);

  Timer RobotTimer = new Timer();  

//  DigitalInput limitswitch1 = new DigitalInput(1);
  
  VictorSPX SPX0 = new VictorSPX(0);

  CANSparkMax Motor0 = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax Motor1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax Motor2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax Motor3 = new CANSparkMax(3, MotorType.kBrushless);

  CANSparkMax Motor4 = new CANSparkMax(4, MotorType.kBrushless);//Elevator Motor
//  CANSparkMax Motor5 = new CANSparkMax(5, MotorType.kBrushless);//12in lift Forward/Backwards Motor
  CANSparkMax Motor6 = new CANSparkMax(6, MotorType.kBrushless);//Arm Angle Motor
  CANSparkMax Motor7 = new CANSparkMax(7, MotorType.kBrushless);//Box Angle Motor

  CANEncoder encoder4 = new CANEncoder(Motor4);//Elevator Encoder
  CANEncoder encoder6 = new CANEncoder(Motor6);//Arm Angle Encoder
  CANEncoder encoder7 = new CANEncoder(Motor7);//Box Angle Encoder

  Solenoid solenoid0 = new Solenoid(5);//Hatch
  Solenoid solenoid1 = new Solenoid(2);//RAM

  DoubleSolenoid solenoid2 = new DoubleSolenoid (1,3);
  DoubleSolenoid.set (DoubleSolenoid.Value.kOff);
  DoubleSolenoid.set (DoubleSolenoid.Value.kForward);
  DoubleSolenoid.set (DoubleSolenoid.Value.kReverse);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Motor2.setInverted(true);
    Motor3.setInverted(true);
    Motor7.setInverted(true);
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
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and 
   * 
   * uncomment the
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
    System.out.println(encoder4.getPosition());

    boolean Eustop = stick2.getRawButtonReleased(5);
    boolean Edstop = stick2.getRawButtonReleased(1);
    boolean Eup = stick2.getRawButtonPressed(5);
    boolean Edown = stick2.getRawButtonPressed(1);
    boolean BoxAdown = stick2.getRawButtonPressed(3);
    boolean BoxAup = stick2.getRawButtonPressed(7);
    boolean BoxAustop = stick2.getRawButtonReleased(7);
    boolean BoxAdstop = stick2.getRawButtonReleased(3);
    boolean RAM = stick.getRawButton(2);
    boolean Hatchout = stick.getRawButtonPressed(3);
    boolean Hatchin = stick.getRawButtonReleased(3);
    boolean climb = stick.getRawButton(3);
    boolean ArmAup = stick2.getRawButtonPressed(6);
    boolean ArmAdown = stick2.getRawButtonPressed(2);
    boolean ArmAustop = stick2.getRawButtonReleased(6);
    boolean ArmAdstop = stick2.getRawButtonReleased(2);
    boolean Climberout = stick2.getRawButtonPressed(4);
    //boolean Climberinout = stick2.getRawButtonPressed();
    
    if(stick.getRawAxis(1) > 0.1 || stick.getRawAxis(1) < -0.1) {
      Motor0.set(stick.getRawAxis(1));
      Motor1.set(stick.getRawAxis(1));
    }
    if(stick.getRawAxis(1) < 0.1 && stick.getRawAxis(1) > -0.1){
      Motor0.set(0);
      Motor1.set(0);
    }
    if(stick.getRawAxis(5) > 0.1 || stick.getRawAxis(5) < -0.1) {
      Motor2.set(stick.getRawAxis(5));
      Motor3.set(stick.getRawAxis(5));
    }
    if(stick.getRawAxis(5) < 0.1 && stick.getRawAxis(5) > -0.1){
      Motor2.set(0);
      Motor3.set(0);
    }

    if(RAM){
      solenoid0.set(true);
    }

    if(Hatchout){
      solenoid1.set(true);
    }
    if(Hatchin){
      solenoid1.set(false);
    }

    if(Climberin){
      solenoid2.set(Value.kOff);
    }

    if(Climberout){
      solenoid2.set(Value.kForward);
    }

    if(Climberinout){
      solenoid2.set(Value.kReverse);
    }

    if(stick.getRawAxis(3) > 0.1){
      SPX0.set(ControlMode.PercentOutput, 0.9);
    }
    if(stick.getRawAxis(2) > 0.1){
      SPX0.set(ControlMode.PercentOutput, -1);
    }
    if(stick.getRawAxis(2) < .1 && stick.getRawAxis(3) < .1){
      SPX0.set(ControlMode.PercentOutput, 0);
    }

    if(Eup){
      Motor4.set(0.88);
    }
    if(Eustop){
      Motor4.set(0);
    }
    if(Edown){
      Motor4.set(-0.88);
    }
    if(Edstop){
      Motor4.set(0);
    }

    if(BoxAup){
      Motor7.set(-0.3);
    }
    if(BoxAustop){
      Motor7.set(0);
    }
    if(BoxAdown){
      Motor7.set(0.3);
    }
    if(BoxAdstop){
      Motor7.set(0);
    }

    if(ArmAup){
      Motor6.set(0.8);
    }
    if(ArmAustop){
      Motor6.set(0);
    }
    if(ArmAdown){
      Motor6.set(-0.5);
    }
    if(ArmAdstop){
      Motor6.set(0);
    }
/*
    if(limitswitch1.get()){
      SRX1.set(ControlMode.PercentOutput, -1);
    }
*/    
/*
    if(climb){
      RobotTimer.start();
      if(RobotTimer.get() < .25){
      solenoid2.set(true);
      }
      else if(RobotTimer.get() > .25 && RobotTimer.get() < 5){
        SRX1.set(ControlMode.PercentOutput, 0.5);
      }
      else if(RobotTimer.get() < 10 && RobotTimer.get() > 5){
        SRX1.set(ControlMode.PercentOutput, 0);
        Motor7.set(0.5);
      }
      else{
        Motor7.set(0);
      }
    }
  

    if(Eup){
      if(encoder7.getPosition() < -9){
      Motor7.set(0.1);
    }
    else if (encoder4.getPosition() > 5){
      Motor7.set(-0.1);
    }
    else{
      Motor7.set(0.0);
      }
    }*/
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
