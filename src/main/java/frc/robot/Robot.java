// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final double WRIST_POSITION_INTAKING = 30;
  private static final double WRIST_POSITION_OUTTAKING = 45;
  private static final double WRIST_POSITION_IDLING = 130;
  private static final double WRIST_GEARING = 60.0 * 2.0;

  private static final double SHOOTER_VELOCITY_IDLING = 600.0 / 60.0 * 360.0;
  private static final double SHOOTER_VELOCITY_SHOOTING = 2000.0 / 60.0 * 360.0;

  private final CANSparkMax queuer = new CANSparkMax(17, MotorType.kBrushless);

  private final CANSparkMax intakeRollers = new CANSparkMax(15, MotorType.kBrushless);

  private final CANSparkMax shooter = new CANSparkMax(18, MotorType.kBrushless);
  private final SparkMaxPIDController shooterPID = shooter.getPIDController();
  private final RelativeEncoder shooterEncoder = shooter.getEncoder();

  private final XboxController controller = new XboxController(0);

  private final CANSparkMax wrist = new CANSparkMax(16, MotorType.kBrushless);
  private final SparkMaxPIDController wristPID = wrist.getPIDController();
  private final RelativeEncoder wristEncoder = wrist.getEncoder();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    wristPID.setP(0.1);
    wristPID.setI(0.0);
    wristPID.setD(0.0);
    wristPID.setIZone(0.0);
    wristPID.setOutputRange(-0.4, 0.4);

    shooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooter.enableVoltageCompensation(10);
    shooter.setSmartCurrentLimit(45);
    shooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    shooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    shooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    shooterPID.setP(0.0005);
    shooterPID.setI(0);
    shooterPID.setD(1);
    shooterPID.setIZone(0);
    shooterPID.setFF(0.00023);
    shooterPID.setOutputRange(0, 1);
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
    SmartDashboard.putNumber("Wrist/Position", wristEncoder.getPosition() / WRIST_GEARING);
    SmartDashboard.putNumber("Shooter/Velocity", shooterEncoder.getVelocity() * 360.0 / 60.0);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double queuerSpeed;
    double intakeRollersSpeed;
    boolean shooting = controller.getRightTriggerAxis() > 0.4;
    boolean intaking = controller.getLeftTriggerAxis() > 0.4;
    boolean outtaking = controller.getLeftBumper();

    if (shooting) {
      shooterPID.setReference(SHOOTER_VELOCITY_SHOOTING * 60.0 / 360.0, ControlType.kVelocity);
      if (shooterEncoder.getVelocity() / 60.0 * 360 >=) { //WHERE I LEFT OFF
        set queuer
      }
    } else if (outtaking) {
      shooterPID.setReference(SHOOTER_VELOCITY_IDLING * 60.0 / 360.0, ControlType.kVelocity);
      queuerSpeed = -0.5;
    } else {
      shooterPID.setReference(SHOOTER_VELOCITY_IDLING * 60.0 / 360.0, ControlType.kVelocity);
      queuerSpeed = 0;
    }

    if (intaking) {
      intakeRollersSpeed = 0.4;
      wristPID.setReference(WRIST_POSITION_INTAKING / 360.0 * WRIST_GEARING, CANSparkMax.ControlType.kPosition);
    } else if (outtaking) {
      intakeRollersSpeed = -0.4;
      wristPID.setReference(WRIST_POSITION_OUTTAKING / 360.0 * WRIST_GEARING, CANSparkMax.ControlType.kPosition);
    } else {
      intakeRollersSpeed = 0;
      wristPID.setReference(WRIST_POSITION_IDLING / 360.0 * WRIST_GEARING, CANSparkMax.ControlType.kPosition);
    }

    queuer.set(queuerSpeed);
    intakeRollers.set(intakeRollersSpeed);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
