/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * methods corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot{
  private static final int liftDeviceID = 3;
  public static final int LoaderPortNum = 8;
  
  private CANSparkMax m_rotateMotor = new CANSparkMax(LoaderPortNum, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private RobotContainer robotContainer;
  private CANSparkMax m_liftMotor;
  private RelativeEncoder m_liftEncoder;
  private XboxController gamePad;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  //BuiltInAccelerometer acceleromenter = new BuiltInAccelerometer();
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  int m_rainbowFirstPixelHue = 0;
  int start = 0;
  double x = 0;

  /**
   * This method is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit()
  {
    // CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);
    m_rotateMotor = new CANSparkMax(liftDeviceID, MotorType.kBrushless);
    gamePad = new XboxController(0);
    robotContainer = new RobotContainer();
    m_pidController = m_rotateMotor.getPIDController();
    m_pidController = m_liftMotor.getPIDController();

      // PID coefficients
    kP = 0.09;
    kI = 0;
    kD = 0.03;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
      
    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  /**
   * This method is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic(){
      // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
      // commands, running already-scheduled commands, removing finished or interrupted commands,
      // and running subsystem periodic() methods.  This must be called from the robot's periodic
      // block in order for anything in the Command-based framework to work.
      CommandScheduler.getInstance().run();
  }

  /**
  * This method is called once each time the robot enters Disabled mode.
  */
  @Override
  public void disabledInit(){
  }

  @Override
  public void disabledPeriodic(){
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit(){
  }

  /**
   * This method is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic(){
  }

  @Override
  public void teleopInit(){
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
  
  }

  /**
  * This method is called periodically during operator control.
  */
  @Override
  public void teleopPeriodic(){
    SmartDashboard.updateValues();

    // encoders
    SmartDashboard.putNumber("Encoder Position", m_liftEncoder.getPosition());
    SmartDashboard.putNumber("Encoder Velocity", m_liftEncoder.getVelocity());
    if (gamePad.getRawButton(6)) {
      m_rotateMotor.set(0.5);
    } else if (gamePad.getRawButton(5)) {
      m_rotateMotor.set(-0.5);
    }else{
      m_rotateMotor.set(0);
    }
    //Heigh Button low
    if (gamePad.getRawButton(1)) {
      m_pidController.setReference(16.66, ControlType.kPosition);
    }else {
      m_rotateMotor.set(0);
    }
  }

  // read PID coefficients from SmartDashboard
  double p = SmartDashboard.getNumber("P Gain", 0);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0);
  double iz = SmartDashboard.getNumber("I Zone", 0);
  double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);
  double rotations = SmartDashboard.getNumber("Set Rotations", 0);
  // controller
  if((p!=kP)) {
    m_pidController.setP(p);
    kP = p;
  }
  if((i!=kI)) {
    m_pidController.setI(i);
    kI = i;
  }
  if((d!=kD)) {
    m_pidController.setD(d);
    kD = d;
  }
  if((iz!=kIz)) {
    m_pidController.setIZone(iz);
    kIz = iz;
  }
  if((ff!=kFF)) {
    m_pidController.setFF(ff);
    kFF = ff;
  }
  if((max!=kMaxOutput)||(min!=kMinOutput)) {
    m_pidController.setOutputRange(min, max);
    kMinOutput = min;
    kMaxOutput = max;
  }
  SmartDashboard.putNumber("SetPoint", rotations);
  SmartDashboard.putNumber("ProcessVariable", m_liftEncoder.getPosition());
 }
}
  @Override
  public void testInit(){
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
  /**
   * This method is called periodically during test mode.
   */
  @Override
  public void testPeriodic(){
      
  }
}

