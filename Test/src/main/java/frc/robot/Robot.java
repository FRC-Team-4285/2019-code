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
import edu.wpi.first.wpilibj.CameraServer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
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
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  Joystick Rattack = new Joystick(5);
  Joystick Lattack = new Joystick(2);
  Joystick stick2 = new Joystick(4);

  Timer RobotTimer = new Timer();  

  int n = 0;

//  DigitalInput limitswitch1 = new DigitalInput(1);

private CANPIDController Motor4PID;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

private CANPIDController Motor7PID;
public double KP, KI, KD, KIz, KFF, KMaxOutput, KMinOutput;

private CANPIDController Motor6PID;
public double p, i, d, iz, ff, MaxO, MinO;

private CANPIDController Motor5PID;
public double P, I, D, IZ, FF, MAXO, MINO;

  VictorSPX SPX0 = new VictorSPX(0);//Ball Intake

  CANSparkMax Motor0 = new CANSparkMax(8, MotorType.kBrushless);//LF
  CANSparkMax Motor1 = new CANSparkMax(1, MotorType.kBrushless);//BF
  CANSparkMax Motor2 = new CANSparkMax(2, MotorType.kBrushless);//FR
  CANSparkMax Motor3 = new CANSparkMax(3, MotorType.kBrushless);//FL

  CANSparkMax Motor4 = new CANSparkMax(4, MotorType.kBrushless);//Elevator Motor
  CANSparkMax Motor5 = new CANSparkMax(5, MotorType.kBrushless);//Lift Motor
  CANSparkMax Motor6 = new CANSparkMax(6, MotorType.kBrushless);//Arm Angle Motor
  CANSparkMax Motor7 = new CANSparkMax(7, MotorType.kBrushless);//Box Angle Motor

  CANEncoder encoder4 = new CANEncoder(Motor4);//Elevator Encoder
  CANEncoder encoder5 = new CANEncoder(Motor5);//Lift Encoder
  CANEncoder encoder6 = new CANEncoder(Motor6);//Arm Angle Encoder
  CANEncoder encoder7 = new CANEncoder(Motor7);//Box Angle Encoder

  Solenoid solenoid1 = new Solenoid(2);//Hatch

  DoubleSolenoid doublesolenoid0 = new DoubleSolenoid(1, 3);//Stepper
  DoubleSolenoid doublesolenoid1 = new DoubleSolenoid(4, 6);
  DoubleSolenoid doublesolenoid2 = new DoubleSolenoid(0, 7);

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

    Motor4PID = Motor4.getPIDController();

    Motor7PID = Motor7.getPIDController();

    Motor6PID = Motor6.getPIDController();

    Motor5PID = Motor5.getPIDController();

    kP = 1;
    kI = 1e-4;
    kD = 3;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 0.3;
    kMinOutput = -0.3;

    KP = 0.1;
    KI = 1e-4;
    KD = 1;
    KIz = 0;
    KFF = 0;
    KMaxOutput = 0.2;
    KMinOutput = -0.2;

    p = 0.1;
    i = 0;
    d = 0;
    iz = 0;
    ff = 0;
    MaxO = 0.2;
    MinO = -0.2;

    P = 0.1;
    I = 0;
    D = 0;
    IZ = 0;
    FF = 0;
    MAXO = 0.3;
    MINO = -0.3;


    Motor4PID.setP(kP);
    Motor4PID.setI(kI);
    Motor4PID.setD(kD);
    Motor4PID.setIZone(kIz);
    Motor4PID.setFF(kFF);
    Motor4PID.setOutputRange(kMinOutput, kMaxOutput);

    Motor7PID.setP(KP);
    Motor7PID.setI(KI);
    Motor7PID.setD(KD);
    Motor7PID.setIZone(KIz);
    Motor7PID.setFF(KFF);
    Motor7PID.setOutputRange(KMinOutput, KMaxOutput);

    Motor6PID.setP(p);
    Motor6PID.setI(i);
    Motor6PID.setD(d);
    Motor6PID.setIZone(iz);
    Motor6PID.setFF(ff);
    Motor6PID.setOutputRange(MinO, MaxO);

    Motor5PID.setP(P);
    Motor5PID.setI(I);
    Motor5PID.setD(D);
    Motor5PID.setIZone(IZ);
    Motor5PID.setFF(FF);
    Motor5PID.setOutputRange(MINO, MAXO);

    solenoid1.set(false);

    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();

    Motor0.setOpenLoopRampRate(0.5);
    Motor1.setOpenLoopRampRate(0.5);
    Motor2.setOpenLoopRampRate(0.5);
    Motor3.setOpenLoopRampRate(0.5);

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
      System.out.println(encoder4.getPosition());

    boolean Eustop = stick2.getRawButtonReleased(5);
    boolean Edstop = stick2.getRawButtonReleased(1);
    boolean Eup = stick2.getRawButtonPressed(5);
    boolean Edown = stick2.getRawButtonPressed(1);

    boolean BoxAdown = stick2.getRawButtonPressed(3);
    boolean BoxAup = stick2.getRawButtonPressed(7);
    boolean BoxAustop = stick2.getRawButtonReleased(7);
    boolean BoxAdstop = stick2.getRawButtonReleased(3);

    boolean Hatchout = Rattack.getRawButtonPressed(2);
    boolean Hatchin = Rattack.getRawButtonReleased(2);
    boolean Stepperout = Rattack.getRawButton(5);
    boolean Stepperin = Lattack.getRawButton(4);
    boolean PLANBout = Rattack.getRawButtonPressed(3);
    boolean PLANBin = Rattack.getRawButtonReleased(3);
    boolean PLANBHout = Lattack.getRawButtonPressed(3);
    boolean PLANBHin = Lattack.getRawButtonReleased(3);

    boolean IntakeRgo = Rattack.getRawButtonPressed(1);
    boolean IntakeRstop = Rattack.getRawButtonReleased(1);
    boolean IntakeLgo = Lattack.getRawButtonPressed(1);
    boolean IntakeLstop = Lattack.getRawButtonReleased(1);

    boolean ArmAup = stick2.getRawButtonPressed(6);
    boolean ArmAdown = stick2.getRawButtonPressed(2);
    boolean ArmAustop = stick2.getRawButtonReleased(6);
    boolean ArmAdstop = stick2.getRawButtonReleased(2);
    
    if(Lattack.getRawAxis(1) > 0.1 || Lattack.getRawAxis(1) < -0.1) {
      Motor0.set(Lattack.getRawAxis(1));
      Motor1.set(Lattack.getRawAxis(1));
    }
    if(Lattack.getRawAxis(1) < 0.1 && Lattack.getRawAxis(1) > -0.1){
      Motor0.set(0);
      Motor1.set(0);
    }
    if(Rattack.getRawAxis(1) > 0.1 || Rattack.getRawAxis(1) < -0.1) {
      Motor2.set(Rattack.getRawAxis(1));
      Motor3.set(Rattack.getRawAxis(1));
    }
    if(Rattack.getRawAxis(1) < 0.1 && Rattack.getRawAxis(1) > -0.1){
      Motor2.set(0);
      Motor3.set(0);
    }

    if(Hatchout){
      solenoid1.set(true);
    }
    if(Hatchin){
      solenoid1.set(false);
    }

    if(PLANBHout){
      doublesolenoid1.set(DoubleSolenoid.Value.kReverse);
    }
    if(PLANBHin){
      doublesolenoid1.set(DoubleSolenoid.Value.kForward);
    }
    
/*    if(PLANBHin){
      doublesolenoid1.set(DoubleSolenoid.Value.kReverse);
    }
*/
    if(PLANBout){
      doublesolenoid2.set(DoubleSolenoid.Value.kForward);
    }
    if(PLANBin){
      doublesolenoid2.set(DoubleSolenoid.Value.kReverse);
    }

    if(Stepperout){
      doublesolenoid0.set(DoubleSolenoid.Value.kForward);
    }
    if(Stepperin){
      doublesolenoid0.set(DoubleSolenoid.Value.kReverse);
    }

    if(IntakeRgo){
      SPX0.set(ControlMode.PercentOutput, 0.9);
    }
    if(IntakeRstop){
      SPX0.set(ControlMode.PercentOutput, 0);
    }
    if(IntakeLgo){
      SPX0.set(ControlMode.PercentOutput, -1);
    }
    if(IntakeLstop){
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
      Motor6.set(-0.7);
    }
    if(ArmAdstop){
      Motor6.set(0);
    }
        // Put default auto code here
        break;
        
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    encoder4.getPosition();
    encoder7.getPosition();
    encoder6.getPosition();
    encoder5.getPosition();
    System.out.println(encoder6.getPosition());

    boolean Hatchout = Rattack.getRawButtonPressed(2);
    boolean Hatchin = Rattack.getRawButtonReleased(2);
    boolean Stepperout = Rattack.getRawButton(5);
    boolean Stepperin = Lattack.getRawButton(4);
    boolean PLANBout = Rattack.getRawButtonPressed(3);
    boolean PLANBin = Rattack.getRawButtonReleased(3);
    boolean PLANBHout = Lattack.getRawButtonPressed(3);
    boolean PLANBHin = Lattack.getRawButtonReleased(3);

    boolean IntakeRgo = Rattack.getRawButtonPressed(1);
    boolean IntakeRstop = Rattack.getRawButtonReleased(1);
    boolean IntakeLgo = Lattack.getRawButtonPressed(1);
    boolean IntakeLstop = Lattack.getRawButtonReleased(1);

    boolean P1 = stick2.getRawButton(1);
    boolean P2 = stick2.getRawButton(2);
    boolean P3 = stick2.getRawButton(3);
    boolean P4 = stick2.getRawButton(4);
    boolean P5 = stick2.getRawButton(5);
    boolean P6 = stick2.getRawButton(6);
    
    if(Lattack.getRawAxis(1) > 0.1 || Lattack.getRawAxis(1) < -0.1) {
      Motor0.set(Lattack.getRawAxis(1));
      Motor1.set(Lattack.getRawAxis(1));
    }
    if(Lattack.getRawAxis(1) < 0.1 && Lattack.getRawAxis(1) > -0.1){
      Motor0.set(0);
      Motor1.set(0);
    }
    if(Rattack.getRawAxis(1) > 0.1 || Rattack.getRawAxis(1) < -0.1) {
      Motor2.set(Rattack.getRawAxis(1));
      Motor3.set(Rattack.getRawAxis(1));
    }
    if(Rattack.getRawAxis(1) < 0.1 && Rattack.getRawAxis(1) > -0.1){
      Motor2.set(0);
      Motor3.set(0);
    }

    if(Hatchout){
      solenoid1.set(true);
    }
    if(Hatchin){
      solenoid1.set(false);
    }

    if(PLANBHout){
      doublesolenoid1.set(DoubleSolenoid.Value.kForward);
    }
    if(PLANBHin){
      doublesolenoid1.set(DoubleSolenoid.Value.kReverse);
    }

    if(PLANBout){
      doublesolenoid2.set(DoubleSolenoid.Value.kForward);
    }
    if(PLANBin){
      doublesolenoid2.set(DoubleSolenoid.Value.kReverse);
    }

    if(Stepperout){
      doublesolenoid0.set(DoubleSolenoid.Value.kForward);
    }
    if(Stepperin){
      doublesolenoid0.set(DoubleSolenoid.Value.kReverse);
    }

    if(IntakeRgo){
      SPX0.set(ControlMode.PercentOutput, 0.9);
    }
    if(IntakeRstop){
      SPX0.set(ControlMode.PercentOutput, 0);
    }
    if(IntakeLgo){
      SPX0.set(ControlMode.PercentOutput, -1);
    }
    if(IntakeLstop){
      SPX0.set(ControlMode.PercentOutput, 0);
    }

    if(P1){
      //Motor4PID.setReference(50, ControlType.kPosition);
      Motor6PID.setReference(0, ControlType.kPosition);
      //Motor7PID.setReference(-5, ControlType.kPosition);
    }

    if(P2){
      //Motor4PID.setReference(100, ControlType.kPosition);
      Motor6PID.setReference(-100, ControlType.kPosition);
      //Motor7PID.setReference(-10, ControlType.kPosition);
    }

    if(P3){
      //Motor4PID.setReference(0, ControlType.kPosition);
      //Motor7PID.setReference(0, ControlType.kPosition);
    }

    if(P4) {
      Motor5PID.setReference(-102, ControlType.kPosition);
    }

    if(P5) {
      Motor5PID.setReference(-51, ControlType.kPosition);
    }

    if(P6) {
      Motor5PID.setReference(0, ControlType.kPosition);
    }

/*
    if(limitswitch1.get()){
      SRX1.set(ControlMode.PercentOutput, -1);
    }    

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
    else if(encoder4.getPosition() > 5){
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
