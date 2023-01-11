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
import edu.wpi.first.math.trajectory.Trajectory.State;
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
  private static final Translation2d frontLeftLocation = new Translation2d(-15, 15);
  private static final Translation2d frontRightLocation = new Translation2d(15, 15);
  private static final Translation2d backLeftLocation = new Translation2d(-15, -15);
  private static final Translation2d backRightLocation = new Translation2d(15, -15);
  private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(frontLeftLocation,
      frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveModule frontLeftModule = new SwerveModule(2, 3, "FrontLeft", new CANCoder(10),
      Rotation2d.fromDegrees(194.6));
  private final SwerveModule frontRightModule = new SwerveModule(4, 5, "FrontRight", new CANCoder(11),
      Rotation2d.fromDegrees(168.95));
  private final SwerveModule backLeftModule = new SwerveModule(6, 7, "BackLeft", new CANCoder(12),
      Rotation2d.fromDegrees(-58.0));
  private final SwerveModule backRightModule = new SwerveModule(8, 9, "BackRight", new CANCoder(13),
      Rotation2d.fromDegrees(27.47));
  // private final Wrist wrist = new Wrist(new CANSparkMax(16, MotorType.kBrushless));
  // private final Shooter shooter = new Shooter(new CANSparkMax(18, MotorType.kBrushless));
  // private final Queuer queuer = new Queuer(new CANSparkMax(17, MotorType.kBrushless));
  // private final IntakeRollers intakeRollers = new IntakeRollers(new CANSparkMax(15, MotorType.kBrushless));
  private final XboxController controller = new XboxController(0);
  // private final Superstructure superstructure = new Superstructure(shooter, wrist, intakeRollers, queuer); 

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean intaking = controller.getLeftTriggerAxis() > 0.5;
    boolean outtaking = controller.getLeftBumper();
    boolean shooting = controller.getRightTriggerAxis() > 0.5;
    // if (intaking) {
    //   superstructure.intaking();
    // } else if (outtaking) {
    //   superstructure.outtaking();
    // } else if (shooting) {
    //   superstructure.shooting();
    // } else {
    //   superstructure.nothingHappening();
    // }
    // set front of the robot to positive Y
    
    ChassisSpeeds speeds = new ChassisSpeeds(controller.getLeftX(), controller.getLeftY(), controller.getRightX());
    SwerveModuleState[] moduleStates = KINEMATICS.toSwerveModuleStates(speeds);
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    frontLeft = CtreModuleState.optimize(frontLeft, frontLeftModule.getRotation());
    frontRight = CtreModuleState.optimize(frontRight, frontRightModule.getRotation());
    backLeft = CtreModuleState.optimize(backLeft, backLeftModule.getRotation());
    backRight = CtreModuleState.optimize(backRight, backRightModule.getRotation());

    frontLeftModule.setAngleAndDrive(frontLeft.angle, frontLeft.speedMetersPerSecond);
    frontRightModule.setAngleAndDrive(frontRight.angle, frontRight.speedMetersPerSecond);
    backLeftModule.setAngleAndDrive(backLeft.angle, backLeft.speedMetersPerSecond);
    backRightModule.setAngleAndDrive(backRight.angle, backRight.speedMetersPerSecond);
  }

  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("Queuer/current", queuer.motor.getOutputCurrent());
    // SmartDashboard.putBoolean("Queuer/sensor", queuer.sensor.get());
    frontLeftModule.log();
    frontRightModule.log();
    backLeftModule.log();
    backRightModule.log();
    // wrist.log();
    // shooter.log();
  }
}
