// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.TimedRobot;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
import edu.wpi.first.wpilibj.XboxController;
>>>>>>> fcadb10dab49022a08cf52914c5dffd82060a68e
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
<<<<<<< HEAD
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;



=======
>>>>>>> fcadb10dab49022a08cf52914c5dffd82060a68e

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private Command m_autonomousCommand;\


  CANSparkMax leftFront = new CANSparkMax(Constants.DrivetrainConstants.kLeftFrontID, MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax (Constants.DrivetrainConstants.kRightFrontID, MotorType.kBrushless);
  CANSparkMax leftRear = new CANSparkMax (Constants.DrivetrainConstants.kLeftRearID, MotorType.kBrushless);
  CANSparkMax rightRear = new CANSparkMax (Constants.DrivetrainConstants.kRightRearID, MotorType.kBrushless);

<<<<<<< HEAD
  //TalonFX shooter1 = new TalonFX(4);
  //TalonFX shooter2 = new TalonFX(21);

  TalonSRX shooter1 = new TalonSRX(4);
  TalonSRX shooter2 = new TalonSRX(21);
=======
  XboxController driver = new XboxController(0);
>>>>>>> fcadb10dab49022a08cf52914c5dffd82060a68e

  


<<<<<<< HEAD

  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  Timer shooterDelay = new Timer();


=======
>>>>>>> fcadb10dab49022a08cf52914c5dffd82060a68e
  // private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();

    leftFront.restoreFactoryDefaults();
    leftRear.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightRear.restoreFactoryDefaults();

<<<<<<< HEAD
    TalonFXConfiguration shooter1_configs = new TalonFXConfiguration();
    TalonFXConfiguration shooter2_configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    //shooter1_configs.MotorOutput.Inverted = ;
    //shooter1_configs.MotorOutput.NeutralMode. = ;



    //shooter1.getConfigurator().apply(shooter1_configs);
    //shooter2.getConfigurator().apply(shooter2_configs);
    shooter1.configFactoryDefault();
    shooter2.configFactoryDefault();


=======
>>>>>>> fcadb10dab49022a08cf52914c5dffd82060a68e
    leftFront.setInverted(true);
    leftRear.setInverted(true);
    rightRear.setInverted(false);
    rightFront.setInverted(false);

    rightRear.follow(rightFront);
    leftRear.follow(leftFront);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    //CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
<<<<<<< HEAD
    // }\
=======
    // }
>>>>>>> fcadb10dab49022a08cf52914c5dffd82060a68e
  }

  /** This function is called periodically during operator control. */
  @Override
<<<<<<< HEAD
  public void teleopPeriodic() {    
    // leftFront.set(-driver.getLeftY());
    // rightFront.set(-driver.getRightY());
    SmartDashboard.putNumber("shooter 1", shooter1.getMotorOutputPercent());
    SmartDashboard.putNumber("shooter 2", shooter2.getMotorOutputPercent());
    if (Math.abs(driver.getLeftY()) > 0.05){
      leftFront.set(-driver.getLeftY());
      rightFront.set(-driver.getLeftY());
    }
    else if (Math.abs(driver.getRightX()) > 0.05){
      leftFront.set(-driver.getRightX());
      rightFront.set(driver.getRightX());
    }
    else {
      leftFront.set(0);
      rightFront.set(0);
    }
    



  if (Math.abs(driver.getLeftTriggerAxis()) > 0.1){
    
    shooter1.set(TalonSRXControlMode.PercentOutput, driver.getLeftTriggerAxis());

    if (shooter1.getMotorOutputPercent() == 1){
      shooter2.set(TalonSRXControlMode.PercentOutput, driver.getLeftTriggerAxis());
    }
    
   // shooter2.set(TalonSRXControlMode.PercentOutput, 20);
    //shooter1.set(TalonSRXControlMode.PercentOutput, 20);
  }
  else if (Math.abs(driver.getRightTriggerAxis()) > 0.1) {
    shooter1.set(TalonSRXControlMode.PercentOutput, -driver.getRightTriggerAxis());
    shooter2.set(TalonSRXControlMode.PercentOutput, -driver.getRightTriggerAxis());
  }
  else {
    shooter1.set(TalonSRXControlMode.PercentOutput, 0);
    shooter2.set(TalonSRXControlMode.PercentOutput, 0);
  }
  if (Math.abs(driver.getLeftTriggerAxis()) > 0.5){
    shooter1.set(TalonSRXControlMode.PercentOutput, 1);
    shooter2.set(TalonSRXControlMode.PercentOutput, 1);
  }
  


  


  

  }

=======
  public void teleopPeriodic() {
    leftFront.set(-driver.getLeftY());
    rightFront.set(-driver.getRightY());
  }
>>>>>>> fcadb10dab49022a08cf52914c5dffd82060a68e


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
