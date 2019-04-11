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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import java.util.TimerTask;

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
  Timer Suction_cup_timer = new Timer(); 

  int piston_repeat;
  int Stepper_variable = 0;

  private CANPIDController Motor5PID;
  public double P, I, D, IZ, FF, MAXO, MINO;

  private CANPIDController motor_elevator_PID;
  public double kp, ki, kd, kiz, kff, kmax, kmin;

  private CANPIDController motor_arm_box_PID;
  public double Kp, Ki, Kd, Kiz, Kff, Kmax, Kmin;

  private CANPIDController suction_cup_lift_PID;
  public double kP, kI, kD, kMAX, kMIN;

  //DigitalInput limitswitch1 = new DigitalInput(0);

  VictorSPX motor_intake = new VictorSPX(0); // Intake Motor - Used with cargo.

  CANSparkMax motor_left_rear = new CANSparkMax(8, MotorType.kBrushless);   // Left Rear Motor
  CANSparkMax motor_left_front = new CANSparkMax(1, MotorType.kBrushless);  // Left Front Motor
  CANSparkMax motor_right_front = new CANSparkMax(2, MotorType.kBrushless); // Right Front Motor
  CANSparkMax motor_right_rear = new CANSparkMax(3, MotorType.kBrushless);  // Right Rear Motor

  CANSparkMax motor_elevator = new CANSparkMax(4, MotorType.kBrushless);  // Elevator Motor
  CANSparkMax motor_lift = new CANSparkMax(5, MotorType.kBrushless);      // Lift Motor
  //CANSparkMax motor_suction_lift = new CANSparkMax(6, MotorType.kBrushless); // Suction cup Motor
  CANSparkMax motor_arm_box = new CANSparkMax(7, MotorType.kBrushless);   //Box Angle Motor

  CANEncoder encoder_elevator = new CANEncoder(motor_elevator);   // Elevator Encoder
  CANEncoder encoder_lift = new CANEncoder(motor_lift);           // Lift Encoder
  CANEncoder encoder_arm_box = new CANEncoder(motor_arm_box);     // Box Angle Encoder
  //CANEncoder encoder_suction_lift = new CANEncoder(motor_suction_lift); //Suction Cup Encodet

  //Solenoid solenoid1 = new Solenoid(2);

  DoubleSolenoid doublesolenoid0 = new DoubleSolenoid(1, 3);// Stepper
  //DoubleSolenoid doublesolenoid1 = new DoubleSolenoid(4, 6);
  //DoubleSolenoid doublesolenoid2 = new DoubleSolenoid(0, 7);
  //DoubleSolenoid suction_cup = new DoubleSolenoid(1, 3); // Suction cup
  //DoubleSolenoid stabilizer = new DoubleSolenoid(2, 5); //Stabilizer, u idiot
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
    MAXO = 0.6;
    MINO = -0.6;

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
    kmax = 0.7;
    kmin = -0.7;

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
    
    /*
    suction_cup_lift_PID = motor_suction_lift.getPIDController();

    kP = 0.1;
    kD = 0;
    kI = 0;
    kMAX = 0.15;
    kMIN = -0.15;

    suction_cup_lift_PID.setP(kP);
    suction_cup_lift_PID.setD(kD);
    suction_cup_lift_PID.setI(kI);
    suction_cup_lift_PID.setOutputRange(kMIN, kMAX);
    */
    //solenoid1.set(false);

    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();

    motor_left_rear.setOpenLoopRampRate(0.3);
    motor_left_front.setOpenLoopRampRate(0.3);
    motor_right_front.setOpenLoopRampRate(0.3);
    motor_right_rear.setOpenLoopRampRate(0.3);

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
      boolean Rocket_level_2 = stick2.getRawButtonPressed(5);
      boolean Cargo_ship = stick2.getRawButtonPressed(1);
  
      boolean Ball_intake_arm = stick2.getRawButtonPressed(3);
      boolean Starting_config = stick2.getRawButtonPressed(7);
      boolean BoxAustop = stick2.getRawButtonReleased(7);
      boolean BoxAdstop = stick2.getRawButtonReleased(3);
  
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
  
      boolean Ball_capture = stick2.getRawButton(2);
      boolean Feeder_Station = stick2.getRawButton(6);
    
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
      
      if (Ball_capture)
    {
      motor_arm_box_PID.setReference(-11, ControlType.kPosition);
      motor_elevator_PID.setReference(-5.6, ControlType.kPosition);
    }
    
    if (Feeder_Station)
    {
      motor_elevator_PID.setReference(134.36, ControlType.kPosition);
      motor_arm_box_PID.setReference(-1.2, ControlType.kPosition);
    }
      /*
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
      */
      /*
      if(Stepperout)
      {
        doublesolenoid0.set(DoubleSolenoid.Value.kForward);
      }
  
      if(Stepperin)
      {
        doublesolenoid0.set(DoubleSolenoid.Value.kReverse);
      }
      */
      if(IntakeRgo)
      {
        motor_intake.set(ControlMode.PercentOutput, 0.75);
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
    
  
    if(Rocket_level_2)
    {
      motor_elevator_PID.setReference(232.36, ControlType.kPosition);
      motor_arm_box_PID.setReference(-0.75, ControlType.kPosition);
      // motor_elevator.set(0.5);
    }
    /*
    if(Eustop)
    {
       motor_elevator.set(0);
    }
    */
    if(Cargo_ship)
    {
      motor_elevator_PID.setReference(202.36, ControlType.kPosition);
      motor_arm_box_PID.setReference(-5, ControlType.kPosition);
      // motor_elevator.set(-0.5);
    }
    /*
    if(Edstop)
    {
      motor_elevator.set(0);
    }
    */
    if(Starting_config)
    {
      motor_arm_box_PID.setReference(-0.75, ControlType.kPosition);
      motor_elevator_PID.setReference(0, ControlType.kPosition);
      Motor5PID.setReference(0, ControlType.kPosition);
      //suction_cup_lift_PID.setReference(0, ControlType.kPosition);
      // motor_arm_box.set(0.1);
    }
    /*
    if(BoxAustop)
    {
      motor_arm_box.set(0);
    }
    */
    if(Ball_intake_arm)
    {
      motor_arm_box_PID.setReference(-13.5, ControlType.kPosition);
      motor_elevator_PID.setReference(-5.6, ControlType.kPosition);
      // motor_arm_box.set(-0.1);
    }
    /*
    if(BoxAdstop)
    {
      motor_arm_box.set(0);
    }  
    */
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
    System.out.println(encoder_arm_box.getPosition());

    Suction_cup_timer.start();

    boolean Eustop = stick2.getRawButtonReleased(5);
    boolean Edstop = stick2.getRawButtonReleased(1);
    boolean Rocket_level_2 = stick2.getRawButtonPressed(5);
    boolean Cargo_ship = stick2.getRawButtonPressed(1);

    boolean Ball_intake_arm = stick2.getRawButtonPressed(3);
    boolean Starting_config = stick2.getRawButtonPressed(7);
    boolean BoxAustop = stick2.getRawButtonReleased(7);
    boolean BoxAdstop = stick2.getRawButtonReleased(3);

    boolean Stepperout = Rattack.getRawButton(5);
    boolean Stepperin = Lattack.getRawButton(4);
    boolean PLANBout = Rattack.getRawButtonPressed(3);
    boolean PLANBin = Rattack.getRawButtonReleased(3);
    boolean PLANBHout = Lattack.getRawButtonPressed(3);
    boolean PLANBHin = Lattack.getRawButtonReleased(3);

    boolean Suction_cup_forward = stick2.getRawButton(4);
    boolean Suction_cup_backward = stick2.getRawButton(8);
    boolean Stabilizer_button = Rattack.getRawButton(8);
    boolean Stabilizer_button_in = Rattack.getRawButton(9);

    boolean IntakeRgo = Rattack.getRawButtonPressed(1);
    boolean IntakeRstop = Rattack.getRawButtonReleased(1);
    boolean IntakeLgo = Lattack.getRawButtonPressed(1);
    boolean IntakeLstop = Lattack.getRawButtonReleased(1);

    boolean Ball_capture = stick2.getRawButton(2);
    boolean Feeder_Station = stick2.getRawButton(6);

    boolean Suction_cup_liftup = Lattack.getRawButtonPressed(8);
    boolean Suction_cup_liftupstop = Lattack.getRawButtonReleased(8);
    boolean Suction_cup_liftdown = Lattack.getRawButtonPressed(9);
    boolean Suction_cup_liftdownstop = Lattack.getRawButtonReleased(9);

    boolean P4 = stick2.getRawButton(4);
    boolean P8 = stick2.getRawButton(8);
    boolean P4stop = stick2.getRawButtonReleased(4);
    boolean P8stop = stick2.getRawButtonReleased(8);
    // boolean Lift_18 = Lattack.getRawButton(6);
    
    
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
      // Motor5PID.setReference(-102, ControlType.kPosition); //For 6"
      // Motor5PID.setReference(-295, ControlType.kPosition); //For 18"
      // Uncomment the following code so you can manually 
      // lower the lift.
      motor_lift.set(-0.4);//Lift Turn down at a speed of -0.1.
    }

    if(P4stop)
    {
      motor_lift.set(0);
    }


    if(P8)
    {
      // Motor5PID.setReference(-25, ControlType.kPosition);
      // Uncomment the following code so you can manually 
      // raise the lift.
      motor_lift.set(0.4);// Lift Turn up at a speed of 0.1.
    }

    if(P8stop)
    {
      motor_lift.set(0);
    }

    /*
    if(Lift_18)
    {
      Motor5PID.setReference(-295, ControlType.kPosition); //For 18"
    }
    */
    
    if(Stepperout)
    {
      doublesolenoid0.set(DoubleSolenoid.Value.kForward);
    }

    if(Stepperin)
    {
      doublesolenoid0.set(DoubleSolenoid.Value.kReverse);
    }
    
    if (Ball_capture)
    {
      motor_arm_box_PID.setReference(-11, ControlType.kPosition);
      motor_elevator_PID.setReference(-5.6, ControlType.kPosition);
    }
    
    if (Feeder_Station)
    {
      motor_elevator_PID.setReference(134.36, ControlType.kPosition);
      motor_arm_box_PID.setReference(-1.2, ControlType.kPosition);
    }

    /*
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
    */

    if(IntakeRgo)
    {
      motor_intake.set(ControlMode.PercentOutput, 0.75);
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
    /*
    if(Suction_cup_liftup)
    {
     // suction_cup_lift_PID.setReference(-22.5, ControlType.kPosition);
     motor_suction_lift.set(1.7);
    }
    
    if(Suction_cup_liftupstop)
    {
      motor_suction_lift.set(0);
    }
    
    if(Suction_cup_liftdown)
    {
      // suction_cup_lift_PID.setReference(5.3, ControlType.kPosition);
      motor_suction_lift.set(-1.7);
    }
    
    if(Suction_cup_liftdownstop)
    {
      motor_suction_lift.set(0);
    }
    */
  
    if(Rocket_level_2)
    {
      motor_elevator_PID.setReference(232.36, ControlType.kPosition);
      motor_arm_box_PID.setReference(-0.75, ControlType.kPosition);
      // motor_elevator.set(0.5);
    }
    /*
    if(Eustop)
    {
       motor_elevator.set(0);
    }
    */
    if(Cargo_ship)
    {
      motor_elevator_PID.setReference(202.36, ControlType.kPosition);
      motor_arm_box_PID.setReference(-5, ControlType.kPosition);
      // motor_elevator.set(-0.5);
    }
    /*
    if(Edstop)
    {
      motor_elevator.set(0);
    }
    */
    if(Starting_config)
    {
      motor_arm_box_PID.setReference(-0.75, ControlType.kPosition);
      motor_elevator_PID.setReference(0, ControlType.kPosition);
      // Motor5PID.setReference(0, ControlType.kPosition);
      // suction_cup_lift_PID.setReference(0, ControlType.kPosition);
      // motor_arm_box.set(0.1);
    }
    /*
    if(BoxAustop)
    {
      motor_arm_box.set(0);
    }
    */
    if(Ball_intake_arm)
    {
      motor_arm_box_PID.setReference(-13.5, ControlType.kPosition);
      motor_elevator_PID.setReference(-5.6, ControlType.kPosition);
      // motor_arm_box.set(-0.1);
    }
    /*
    if(BoxAdstop)
    {
      motor_arm_box.set(0);
    }  
    */
    /*
    if(Stabilizer_button)
    {
      stabilizer.set(DoubleSolenoid.Value.kReverse);
    }

    if(Stabilizer_button_in)
    {
      stabilizer.set(DoubleSolenoid.Value.kForward);
    }

    if(Suction_cup_backward)
    {
      suction_cup.set(DoubleSolenoid.Value.kForward);
    }

    if(Suction_cup_forward)
    {
      suction_cup.set(DoubleSolenoid.Value.kReverse);
    }
    */
/*
    RobotTimer.start();
    if(Suction_cup_button)
    {
      for (piston_repeat = 0; piston_repeat < 4; piston_repeat++)
      {
        RobotTimer.reset();
        if (RobotTimer.get() < 0.5)
        {
          suction_cup.set(DoubleSolenoid.Value.kForward);
        }
        if (RobotTimer.get() > 0.5)
        {
          suction_cup.set(DoubleSolenoid.Value.kReverse);
        }
      }
    }
*/
/*
    if(stick2.getRawAxis(1) > 0.9)
    {
      TimerTask Suction_task = new TimerTask()
    {
    @Override
    public void run() 
    {
      Suction_cup_timer.reset();

        if (Suction_cup_timer.get() < 0.25)
        {
          suction_cup.set(DoubleSolenoid.Value.kForward);
        }

        if (Suction_cup_timer.get() > 0.25)
        {
          suction_cup.set(DoubleSolenoid.Value.kReverse);
        }   
     }
    };
    
    RobotTimer.schedule(Suction_task, 2000, 2000);

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
  

    if(Rocket_level_2)
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
