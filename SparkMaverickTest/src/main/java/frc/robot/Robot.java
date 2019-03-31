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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

/* mfwass: These are unused imports that Drew blew a cap about when I removed them.
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.CAN;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
*/

/**
 * The VM is configured to automatically run this class, and to call the
 * functions comotor_right_rearesponding to each mode, as described in the IterativeRobot
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

  int piston_repeat;

  private CANPIDController Motor5PID;
  public double P, I, D, IZ, FF, MAXO, MINO;

  private CANPIDController motor_elevator_PID;
  public double kp, ki, kd, kiz, kff, kmax, kmin;

  private CANPIDController motor_arm_box_PID;
  public double Kp, Ki, Kd, Kiz, Kff, Kmax, Kmin;

  //DigitalInput limitswitch1 = new DigitalInput(0);

  VictorSPX motor_intake = new VictorSPX(0); // Intake Motor - Used with cargo.

  CANSparkMax motor_left_rear = new CANSparkMax(8, MotorType.kBrushless);   // Left Rear Motor
  CANSparkMax motor_left_front = new CANSparkMax(1, MotorType.kBrushless);  // Left Front Motor
  CANSparkMax motor_right_front = new CANSparkMax(2, MotorType.kBrushless); // Right Front Motor
  CANSparkMax motor_right_rear = new CANSparkMax(3, MotorType.kBrushless);  // Right Rear Motor

  CANSparkMax motor_elevator = new CANSparkMax(4, MotorType.kBrushless);  //Elevator Motor
  CANSparkMax motor_lift = new CANSparkMax(5, MotorType.kBrushless);      //Lift Motor
  CANSparkMax motor_arm_box = new CANSparkMax(7, MotorType.kBrushless);   //Box Angle Motor

  CANEncoder encoder_elevator = new CANEncoder(motor_elevator);   //Elevator Encoder
  CANEncoder encoder_lift = new CANEncoder(motor_lift);           //Lift Encoder
  CANEncoder encoder_arm_box = new CANEncoder(motor_arm_box);     //Box Angle Encoder

  Solenoid solenoid1 = new Solenoid(2);//Hatch

  DoubleSolenoid doublesolenoid0 = new DoubleSolenoid(1, 3);//Stepper
  DoubleSolenoid doublesolenoid1 = new DoubleSolenoid(4, 6);
  DoubleSolenoid doublesolenoid2 = new DoubleSolenoid(0, 7);
  //DoubleSolenoid suction_cup = new DoubleSolenoid(2, 5);
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Motor5PID = motor_lift.getPIDController();

    P = 0.1;
    I = 0;
    D = 0;
    IZ = 0;
    FF = 0;
    MAXO = 0.3;
    MINO = -0.3;

    Motor5PID.setP(P);
    Motor5PID.setI(I);
    Motor5PID.setD(D);
    Motor5PID.setIZone(IZ);
    Motor5PID.setFF(FF);
    Motor5PID.setOutputRange(MINO, MAXO);

    motor_elevator_PID = motor_elevator.getPIDController();

    kp = 0.1;
    ki = 0;
    kd = 0;
    kiz = 0;
    kff = 0;
    kmax = 1;
    kmin = -1;

    motor_elevator_PID.setP(kp);
    motor_elevator_PID.setI(ki);
    motor_elevator_PID.setD(kd);
    motor_elevator_PID.setIZone(kiz);
    motor_elevator_PID.setFF(kff);
    motor_elevator_PID.setOutputRange(kmin, kmax);

    motor_arm_box_PID = motor_arm_box.getPIDController();

    Kp = 0.1;
    Ki = 0;
    Kd = 0;
    Kiz = 0;
    Kff = 0;
    Kmax = 0.2;
    Kmin = -0.15;

    motor_arm_box_PID.setP(Kp);
    motor_arm_box_PID.setI(Ki);
    motor_arm_box_PID.setD(Kd);
    motor_arm_box_PID.setIZone(Kiz);
    motor_arm_box_PID.setFF(Kff);
    motor_arm_box_PID.setOutputRange(Kmin, Kmax);

    solenoid1.set(false);

    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();

    motor_left_rear.setOpenLoopRampRate(1);
    motor_left_front.setOpenLoopRampRate(1);
    motor_right_front.setOpenLoopRampRate(1);
    motor_right_rear.setOpenLoopRampRate(1);

    motor_right_front.setInverted(true);
    motor_right_rear.setInverted(true);
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
      System.out.println(encoder_elevator.getPosition());

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
    
    if(Lattack.getRawAxis(1) > 0.1 || Lattack.getRawAxis(1) < -0.1) 
    {
      motor_left_rear.set(Lattack.getRawAxis(1));
      motor_left_front.set(Lattack.getRawAxis(1));
    }

    if(Lattack.getRawAxis(1) < 0.1 && Lattack.getRawAxis(1) > -0.1)
    {
      motor_left_rear.set(0);
      motor_left_front.set(0);
    }

    if(Rattack.getRawAxis(1) > 0.1 || Rattack.getRawAxis(1) < -0.1) 
    {
      motor_right_front.set(Rattack.getRawAxis(1));
      motor_right_rear.set(Rattack.getRawAxis(1));
    }

    if(Rattack.getRawAxis(1) < 0.1 && Rattack.getRawAxis(1) > -0.1)
    {
      motor_right_front.set(0);
      motor_right_rear.set(0);
    }


    if(Hatchout)
    {
      solenoid1.set(true);
    }

    if(Hatchin)
    {
      solenoid1.set(false);
    }

    if(PLANBHout)
    {
      doublesolenoid1.set(DoubleSolenoid.Value.kForward);
    }

    if(PLANBHin)
    {
      doublesolenoid1.set(DoubleSolenoid.Value.kReverse);
    }
    
    if(PLANBout)
    {
      doublesolenoid2.set(DoubleSolenoid.Value.kForward);
    }

    if(PLANBin)
    {
      doublesolenoid2.set(DoubleSolenoid.Value.kReverse);
    }

    if(Stepperout)
    {
      doublesolenoid0.set(DoubleSolenoid.Value.kForward);
    }

    if(Stepperin)
    {
      doublesolenoid0.set(DoubleSolenoid.Value.kReverse);
    }

    if(IntakeRgo)
    {
      motor_intake.set(ControlMode.PercentOutput, 1);
    }

    if(IntakeRstop)
    {
      motor_intake.set(ControlMode.PercentOutput, 0);
    }

    if(IntakeLgo)
    {
      motor_intake.set(ControlMode.PercentOutput, -1);
    }

    if(IntakeLstop)
    {
      motor_intake.set(ControlMode.PercentOutput, 0);
    }

    if(Eup)
    {
      motor_elevator.set(0.88);
    }

    if(Eustop)
    {
      motor_elevator.set(0);
    }

    if(Edown)
    {
      motor_elevator.set(-0.88);
    }

    if(Edstop)
    {
      motor_elevator.set(0);
    }

    if(BoxAup)
    {
      motor_arm_box.set(0.05);
    }

    if(BoxAustop)
    {
      motor_arm_box.set(0);
    }

    if(BoxAdown)
    {
      motor_arm_box.set(-0.05);
    }

    if(BoxAdstop)
    {
      motor_arm_box.set(0);
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
    //System.out.println(encoder_lift.getPosition());
    System.out.println(encoder_elevator.getPosition());

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

    boolean Elevator_test_up = stick2.getRawButton(2);
    boolean Elevator_test_down = stick2.getRawButton(6);

    boolean P4 = stick2.getRawButton(4);
    boolean P8 = stick2.getRawButton(8);
    boolean P4stop = stick2.getRawButtonReleased(4);
    boolean P8stop = stick2.getRawButtonReleased(8);
    
    
    if(Lattack.getRawAxis(1) > 0.1 || Lattack.getRawAxis(1) < -0.1) 
    {
      motor_left_rear.set(Lattack.getRawAxis(1));
      motor_left_front.set(Lattack.getRawAxis(1));
    }

    if(Lattack.getRawAxis(1) < 0.1 && Lattack.getRawAxis(1) > -0.1)
    {
      motor_left_rear.set(0);
      motor_left_front.set(0);
    }

    if(Rattack.getRawAxis(1) > 0.1 || Rattack.getRawAxis(1) < -0.1) 
    {
      motor_right_front.set(Rattack.getRawAxis(1));
      motor_right_rear.set(Rattack.getRawAxis(1));
    }

    if(Rattack.getRawAxis(1) < 0.1 && Rattack.getRawAxis(1) > -0.1)
    {
      motor_right_front.set(0);
      motor_right_rear.set(0);
    }
    
    if(P4)
    {
       Motor5PID.setReference(-102, ControlType.kPosition);
      // Uncomment the following code so you can manually 
      // lower the lift.
      // motor_lift.set(-0.2);//Lift Turn down at a speed of -0.1.
    }

    if(P8)
    {
       Motor5PID.setReference(0, ControlType.kPosition);
      // Uncomment the following code so you can manually 
      // raise the lift.
      // motor_lift.set(0.2);// Lift Turn up at a speed of 0.1.
    }
    /*
    if(P4stop){
      motor_lift.set(0);
    }
    if(P8stop){
      motor_lift.set(0);
    }
    */
    if (Elevator_test_up)
    {
      motor_elevator_PID.setReference(0, ControlType.kPosition);
      motor_arm_box_PID.setReference(-12.5, ControlType.kPosition);
    }
    
    if (Elevator_test_down)
    {
      motor_elevator_PID.setReference(350, ControlType.kPosition);
      motor_arm_box_PID.setReference(-3.2, ControlType.kPosition);
    }

    if(Hatchout)
    {
      solenoid1.set(true);
    }
   
    if(Hatchin)
    {
      solenoid1.set(false);
    }

    if(PLANBHout)
    {
      doublesolenoid1.set(DoubleSolenoid.Value.kForward);
    }
    
    if(PLANBHin)
    {
      doublesolenoid1.set(DoubleSolenoid.Value.kReverse);
    }

    if(PLANBout)
    {
      doublesolenoid2.set(DoubleSolenoid.Value.kForward);
    }

    if(PLANBin)
    {
      doublesolenoid2.set(DoubleSolenoid.Value.kReverse);
    }

    if(Stepperout)
    {
      doublesolenoid0.set(DoubleSolenoid.Value.kForward);
    }

    if(Stepperin)
    {
      doublesolenoid0.set(DoubleSolenoid.Value.kReverse);
    }

    if(IntakeRgo)
    {
      motor_intake.set(ControlMode.PercentOutput, 1);
    }

    if(IntakeRstop)
    {
      motor_intake.set(ControlMode.PercentOutput, 0);
    }

    if(IntakeLgo)
    {
      motor_intake.set(ControlMode.PercentOutput, -1);
    }
    
    if(IntakeLstop)
    {
      motor_intake.set(ControlMode.PercentOutput, 0);
    }

    if(Eup)
    {
      motor_elevator_PID.setReference(595, ControlType.kPosition);
      motor_arm_box_PID.setReference(-0.75, ControlType.kPosition);
      //motor_elevator.set(0.88);
    }
/*
    if(Eustop)
    {
      motor_elevator.set(0);
    }
*/
    if(Edown)
    {
      motor_elevator_PID.setReference(520, ControlType.kPosition);
      motor_arm_box_PID.setReference(-4.2, ControlType.kPosition);
    }
/*
    if(Edstop)
    {
      motor_elevator.set(0);
    }
*/
    if(BoxAup)
    {
      motor_arm_box_PID.setReference(-0.75, ControlType.kPosition);
      motor_elevator_PID.setReference(0, ControlType.kPosition);
      //motor_arm_box.set(0.1);
    }
/*
    if(BoxAustop)
    {
      motor_arm_box.set(0);
    }
*/
    if(BoxAdown)
    {
      motor_arm_box_PID.setReference(-14, ControlType.kPosition);
      //motor_arm_box.set(-0.1);
    }
/*
    if(stick2.getRawAxis(1) > 0.9)
    {
      for (piston_repeat = 0; piston_repeat < 4; piston_repeat++)
      {
        RobotTimer.start();
        if (RobotTimer.get() < 0.5)
        {
          suction_cup.set(DoubleSolenoid.Value.kForward);
        }
        if (RobotTimer.get() > 0.5)
        {
          suction_cup.set(DoubleSolenoid.Value.kReverse);
        }
        RobotTimer.reset();
      }
    }
/*
   if(BoxAdstop)
    {
      motor_arm_box.set(0);
    }
    
/*
    if(climb)
    {
      RobotTimer.start();
      if(RobotTimer.get() < .25){
      solenoid2.set(true);
     }
      else if(RobotTimer.get() > .25 && RobotTimer.get() < 5)
      {
        SRX1.set(ControlMode.PercentOutput, 0.5);
      }


      else if(RobotTimer.get() < 10 && RobotTimer.get() > 5)
      {
        SRX1.set(ControlMode.PercentOutput, 0);
        motor_arm_box.set(0.5);
      }
      else{
        motor_arm_box.set(0);
      }
    }
  

    if(Eup)
    {
      if(encoder_arm_box.getPosition() < -9){
      motor_arm_box.set(0.1);
    }
    else if (encoder_elevator.getPosition() > 5)
    {
      motor_arm_box.set(-0.1);
    }
    else
    {
      motor_arm_box.set(0.0);
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
