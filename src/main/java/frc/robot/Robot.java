// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import org.w3c.dom.xpath.XPathResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /** 64:1 gearing reduction, 2:1 chain reduction. */
  private static final double WRIST_GEARING = 60 * 2;
  private static final double WRIST_POSITION_OUTTAKING = 60.0 / 360.0 * WRIST_GEARING;
  private static final double WRIST_POSITION_INTAKING = 40.0 / 360.0 * WRIST_GEARING;
  private static final double WRIST_POSITION_IDLE = 110.0 / 360.0 * WRIST_GEARING;
  private static final double SHOOTER_SHOOTING_VELOCITY = 2000 / 60.0 * 360.0;
  private static final double SHOOTING_IDLE = 800 / 60.0 * 360.0 * 0;
  private static final double ticksPerRotation = 12.8 * 2048;

  private static final Translation2d frontLeftLocation = new Translation2d(-15, 15);
  private static final Translation2d frontRightLocation = new Translation2d(15, 15);
  private static final Translation2d backLeftLocation = new Translation2d(-15, -15);
  private static final Translation2d backRightLocation = new Translation2d(15, -15);
  private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(frontLeftLocation,
      frontRightLocation, backLeftLocation, backRightLocation);

  private final CANSparkMax intakeRollers = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax wrist = new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax shooter = new CANSparkMax(18, MotorType.kBrushless);
  private final CANSparkMax queuer = new CANSparkMax(17, MotorType.kBrushless);
  private final XboxController controller = new XboxController(0);
  private final SparkMaxPIDController wristPid = wrist.getPIDController();
  private final RelativeEncoder wristEncoder = wrist.getEncoder();
  private final SparkMaxPIDController shooterPid = shooter.getPIDController();
  private final RelativeEncoder shootingEncoder = shooter.getEncoder();
  private final DigitalInput queuerSensor = new DigitalInput(0);

  private final SwerveModule frontLeftModule = new SwerveModule(2, 3, "FrontLeft", new CANCoder(10),
     Rotation2d.fromDegrees(194.6));
  private final SwerveModule frontRightModule = new SwerveModule(4, 5, "FrontRight", new CANCoder(11),
     Rotation2d.fromDegrees(168.95));
  private final SwerveModule backLeftModule = new SwerveModule(6, 7, "BackLeft", new CANCoder(12), 
   Rotation2d.fromDegrees(-58.0));
  private final SwerveModule backRightModule = new SwerveModule(8, 9, "BackRight", new CANCoder(13),
     Rotation2d.fromDegrees(27.47));

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    wristPid.setP(0.3);
    wristPid.setI(0);
    wristPid.setD(0);
    wristPid.setIZone(0);
    wristPid.setFF(0);
    wristPid.setOutputRange(-1, 1);

    intakeRollers.setInverted(true);

    shooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooter.enableVoltageCompensation(10);
    shooter.setSmartCurrentLimit(45);
    shooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    shooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    shooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    shooterPid.setP(0.005);
    shooterPid.setI(0);
    shooterPid.setD(1);
    shooterPid.setIZone(0);
    shooterPid.setFF(0.00023);
    shooterPid.setOutputRange(0, 1);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean intaking = controller.getLeftTriggerAxis() > 0.5;
    boolean outtaking = controller.getLeftBumper();
    boolean shooting = controller.getRightTriggerAxis() > 0.5;
    if (intaking) {
      intakeRollers.set(0.4);
      shooterPid.setReference(SHOOTING_IDLE * 60.0 / 360.0, ControlType.kVelocity);
      wristPid.setReference(WRIST_POSITION_INTAKING, ControlType.kPosition);
      if (queuerSensor.get()) {
        queuer.set(0);
      } else {
        queuer.set(0.25);
      }
    } else if (outtaking) {
      intakeRollers.set(-0.4);
      shooterPid.setReference(SHOOTING_IDLE * 60.0 / 360.0, ControlType.kVelocity);
      queuer.set(-0.5);
      wristPid.setReference(WRIST_POSITION_OUTTAKING, ControlType.kPosition);
    } else if (shooting) {
      shooterPid.setReference(SHOOTER_SHOOTING_VELOCITY * 60.0 / 360.0, ControlType.kVelocity);
      wristPid.setReference(WRIST_POSITION_INTAKING, ControlType.kPosition);
      if (shootingEncoder.getVelocity() / 60.0 * 360.0 >= SHOOTER_SHOOTING_VELOCITY * 0.95) {
        intakeRollers.set(0.4);
        queuer.set(0.5);
      } else {
        intakeRollers.set(0);
        queuer.set(0);
      }
    } else {
      shooterPid.setReference(SHOOTING_IDLE * 60.0 / 360.0, ControlType.kVelocity);
      queuer.set(0);
      intakeRollers.set(0);
      wristPid.setReference(WRIST_POSITION_IDLE, ControlType.kPosition);
    }
    // set front of the robot to positive Y
    ChassisSpeeds speeds = new ChassisSpeeds(controller.getLeftX(), controller.getLeftY(), controller.getRightX());
    SwerveModuleState[] moduleStates = KINEMATICS.toSwerveModuleStates(speeds);
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    frontLeftModule.setAngleAndDrive(frontLeft.angle, frontLeft.speedMetersPerSecond);
    frontRightModule.setAngleAndDrive(frontRight.angle, frontRight.speedMetersPerSecond);
    backLeftModule.setAngleAndDrive(backLeft.angle, backLeft.speedMetersPerSecond);
    backRightModule.setAngleAndDrive(backRight.angle, backRight.speedMetersPerSecond);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Queuer/current", queuer.getOutputCurrent());
    SmartDashboard.putNumber("wrist/angle", wristEncoder.getPosition() * 360.0 / WRIST_GEARING);
    SmartDashboard.putNumber("Shooter/Velocity", shootingEncoder.getVelocity() / 60.0 * 360.0);
    SmartDashboard.putBoolean("Queuer/sensor", queuerSensor.get());
    frontLeftModule.log();
    frontRightModule.log();
    backLeftModule.log();
    backRightModule.log();
  }
}
