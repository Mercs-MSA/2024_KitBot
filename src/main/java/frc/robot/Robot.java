// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;





/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private Command m_autonomousCommand;\

  private final DigitalInput noteSensor = new DigitalInput(8);

  CANSparkMax leftFront = new CANSparkMax(Constants.DrivetrainConstants.kLeftFrontID, MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax (Constants.DrivetrainConstants.kRightFrontID, MotorType.kBrushless);
  CANSparkMax leftRear = new CANSparkMax (Constants.DrivetrainConstants.kLeftRearID, MotorType.kBrushless);
  CANSparkMax rightRear = new CANSparkMax (Constants.DrivetrainConstants.kRightRearID, MotorType.kBrushless);
  CANSparkMax climber = new CANSparkMax (Constants.DrivetrainConstants.kClimberID, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(Constants.DrivetrainConstants.kIntakeID, MotorType.kBrushless);

  //TalonFX shooter1 = new TalonFX(4);
  //TalonFX shooter2 = new TalonFX(21);

  TalonSRX shooter1 = new TalonSRX(4);
  TalonSRX shooter2 = new TalonSRX(21);
  TalonSRX amp = new TalonSRX(16);
  
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  Timer shooterDelay = new Timer();

  Timer autoTimer =  new Timer();

  boolean shooterDelayReached = false;

  boolean precisonMode = false;

  final double precisionSpeed = 0.35;

  double m_setpoint;

  boolean noteLoaded = false;

  boolean driveDir = true;

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
    climber.restoreFactoryDefaults();

    TalonFXConfiguration shooter1_configs = new TalonFXConfiguration();
    TalonFXConfiguration shooter2_configs = new TalonFXConfiguration();
    TalonFXConfiguration anp_configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-em 5f of the motor */
    //shooter1_configs.MotorOutput.Inverted = ;
    //shooter1_configs.MotorOutput.NeutralMode. = ;



    //shooter1.getConfigurator().apply(shooter1_configs);
    //shooter2.getConfigurator().apply(shooter2_configs);
    shooter1.configFactoryDefault();
    shooter2.configFactoryDefault();

    m_pidController = climber.getPIDController();

    // Encoder object created to display position values
    m_encoder = climber.getEncoder();

    // PID coefficients
    kP = 0.5; 
    kI = 0; //1e-4;
    kD = 0; //1; 
    kIz = 0; 
    kFF = 0; 

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

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
    SmartDashboard.putNumber("shooter 1", shooter1.getMotorOutputPercent());
    SmartDashboard.putNumber("shooter 2", shooter2.getMotorOutputPercent());
    SmartDashboard.putBoolean("precisonMode", precisonMode);
    SmartDashboard.putNumber("climber encoder position", climber.getEncoder().getPosition());
    SmartDashboard.putBoolean("shooter loaded?", !noteSensor.get());
    SmartDashboard.putNumber("Auton timer", autoTimer.get());
    SmartDashboard.putNumber("shooter delay timer", shooterDelay.get());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoTimer.reset();
    autoTimer.start();
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  if (autoTimer.get() < 2) { //Starts shooter
  shooter1.set(TalonSRXControlMode.PercentOutput, 1);
  shooterDelay.start();
  if (shooterDelay.get() > 0.4) { //Shoots pre-loaded note
      shooter2.set(TalonSRXControlMode.PercentOutput, 1);
      shooterDelay.reset();
    } else {
      shooterDelayReached = false;
    }
  } else if (autoTimer.get() < 4) { //Parks
    shooter1.set(TalonSRXControlMode.PercentOutput, 0);
    shooter2.set(TalonSRXControlMode.PercentOutput, 0);
      rightFront.set(-0.10);
      rightRear.set(-0.10);
      leftFront.set(-0.10);
      leftRear.set(-0.10);
  // } else {
  //   rightFront.set(0);
  //   rightRear.set(0);
  //   leftFront.set(0);
  //   leftRear.set(0);
  // Experimental code vvv
  } else if (autoTimer.get() < 6 ) { //Retrieves second note through floor intake
      rightFront.set(-0.10);
      rightRear.set(-0.10);
      leftFront.set(-0.10);
      leftRear.set(-0.10);
      intake.set(-0.6);
      } else if (autoTimer.get() < 7 ) { //Goes back to score
        rightFront.set(0.2);
        rightRear.set(0.2);
        leftFront.set(0.2);
        leftRear.set(0.2);
        intake.set(0);
      } else if (autoTimer.get() < 11) { //Shoots second note
        rightFront.set(0);
        rightRear.set(0);
        leftFront.set(0);
        leftRear.set(0);        
        shooter1.set(TalonSRXControlMode.PercentOutput, 1);
        shooterDelay.start();
        if (shooterDelay.get() > 0.5) { 
            shooter2.set(TalonSRXControlMode.PercentOutput, 1);
          } else {
            shooterDelayReached = false;
          }
        } else if (autoTimer.get() < 12) { //Parks
          rightFront.set(-0.10);
          rightRear.set(-0.10);
          leftFront.set(-0.10);
          leftRear.set(-0.10);
      } else if (autoTimer.get() > 12.01) {
        shooter1.set(TalonSRXControlMode.PercentOutput, 0);
        shooter2.set(TalonSRXControlMode.PercentOutput, 0);
        rightFront.set(0);
        rightRear.set(0);
        leftFront.set(0);
        leftRear.set(0);
    }

}

// shooterDelay.start();
//     if (shooterDelay.get() > 0.5){
//       shooterDelayReached = true;
//     }
//     else {
//       shooterDelayReached = false;
//     }


  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }\
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {    
     /*this is the start of code for tank drive */

    // if (Math.abs(driver.getLeftY()) > 0.1) {
    //   leftFront.set(driver.getLeftY());
    // }

    // else if (Math.abs(driver.getRightY()) > 0.1){
    //   rightFront.set(driver.getRightX());
    // }

    /*this is the end of the code for tank drive */

    /*start of arcade drive */

    // if ((precisonMode == false) && (driver.getRawButton(7))) {
    //     precisonMode = true;
    // }
    // else if ((precisonMode == true) && (driver.getRawButton(7))) {
    //     precisonMode = false;
    // }
    // if (precisonMode) {
    //   if (Math.abs(driver.getLeftY()) > 0.1){
    //     leftFront.set(-1*precisionSpeed*driver.getLeftY());
    //     rightFront.set(-1*precisionSpeed*driver.getLeftY());
    //   }
    //   else if (Math.abs(driver.getRightX()) > 0.1){
    //     leftFront.set(-1*precisionSpeed*driver.getRightX());
    //     rightFront.set(precisionSpeed*driver.getRightX());
    //   }
    //   else {
    //     leftFront.set(0);
    //     rightFront.set(0);
    //   }
    // } else {
  
      //ABDULLAH DRIVE CODE

      if (Math.abs(driver.getLeftY()) > 0.1) { //If we are trying to MOVE
        
        if (Math.abs(driver.getRightX()) > 0.1) { //If we are trying to move + turn (CURVE)
          
          if (driver.getLeftY() < 0) { //Curving FORWARDS
            driveDir = true;
            if (driver.getRightX() > 0) { //If we are trying to curve RIGHT

              rightFront.set(driver.getRightX()/3);
              leftFront.set(0.1);
            } else { //If we are trying to curve LEFT

              rightFront.set(0.1);
              leftFront.set(Math.abs(driver.getRightX()/3));
            }
          } else { //If we are trying to curve BACKWARDS
            driveDir = false;
            if (driver.getRightX() > 0) { //If we are trying to curve RIGHT

              rightFront.set(-driver.getRightX()/3);
              leftFront.set(-0.1);
            } else { //If we are trying to curve LEFT

              rightFront.set(-0.1);
              leftFront.set(-Math.abs(driver.getRightX()/3));
            }
          }
        } else { //If we are trying NOT trying to turn (FORWARD/BACKWARD)
          driveDir = true;
          leftFront.set(-driver.getLeftY()/4);
          rightFront.set(-driver.getLeftY()/4);
        }
      } else if (Math.abs(driver.getRightX()) > 0.1) { //If we are trying to ROTATE (LEFT/RIGHT)
          if (driveDir) {
            rightFront.set(driver.getRightX()/4);
            leftFront.set(-driver.getRightX()/4);
          } else {
            rightFront.set(-driver.getRightX()/4);
            leftFront.set(driver.getRightX()/4);
          }
          
      } else { //If we are trying NOT trying to move (STATIONARY)

          leftFront.set(0);
          rightFront.set(0);
      } 



  //   /*end of arcade drive */

  if (driver.getRawButton(5)){
    shooter1.set(TalonSRXControlMode.PercentOutput, -0.75);
    shooter2.set(TalonSRXControlMode.PercentOutput, -0.75);
  }




  else if (driver.getRawButton(6)){
    shooter1.set(TalonSRXControlMode.PercentOutput, 1);
    shooterDelay.start();
    if (shooterDelay.get() > 0.5){
      shooterDelayReached = true;
    }
    else {
      shooterDelayReached = false;
    }
    if (shooterDelayReached){
      shooter2.set(TalonSRXControlMode.PercentOutput, 1);
    }
    else {

      shooter2.set(TalonSRXControlMode.PercentOutput, 0);
    }


  }
  else {
    shooter1.set(TalonSRXControlMode.PercentOutput, 0);
    shooter2.set(TalonSRXControlMode.PercentOutput, 0);
    shooterDelay.reset();
    shooterDelayReached = false;
  }
  
    if (driver.getRawButton(1)){
    amp.set(TalonSRXControlMode.PercentOutput,-1);
     } else if (driver.getRawButton(3)){
    amp.set(TalonSRXControlMode.PercentOutput,0.35);
  }
  // This section is for ____ idk what this is - atharv
    else {
    amp.set(TalonSRXControlMode.PercentOutput,0);
    }
  

  //Climber code
   if (driver.getPOV()==0){
  climber.set(-1);
   }
  else if (driver.getPOV()==180) {
    climber.set(1);
  }  else {
  climber.set(0);
   }

  //intake code by yours truly srihan

  if (driver.getRightTriggerAxis() > 0.2) {

    intake.set(-0.6);
  } else if (driver.getLeftTriggerAxis() > 0.2) {

    intake.set(0.4);
  }  else {
// we should move the intake to the operator controller
    intake.set(0);
  }

}
  
  

    

    @Override
  public void testInit() {}
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}



  // This function is called periodically whilst in simulation. */
   @Override
   public void simulationPeriodic() {}

}

