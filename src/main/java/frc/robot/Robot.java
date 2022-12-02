// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
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
  private static final double GEARING = 64 * 2;
  private static final double WRIST_POSITION_OUTTAKING = 50.0 / 360.0 * GEARING;
  private static final double WRIST_POSITION_INTAKING = 30.0 / 360.0 * GEARING;
  private static final double WRIST_POSITION_IDLE = 110.0 / 360.0 * GEARING;
  private final CANSparkMax intakeRollers = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax wrist = new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax shooter = new CANSparkMax(18, MotorType.kBrushless);
  private final CANSparkMax queuer = new CANSparkMax(17, MotorType.kBrushless);
  private final XboxController controller = new XboxController(0);
  private final SparkMaxPIDController wristPid = wrist.getPIDController();
  private final RelativeEncoder wristEncoder = wrist.getEncoder();

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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean intaking = controller.getLeftTriggerAxis() > 0.5;
    boolean outtaking = controller.getLeftBumper();
    boolean shooting = controller.getRightTriggerAxis() > 0.5;
    if (intaking) {
      intakeRollers.set(0.4);
      queuer.set(0.5);
      shooter.set(0);
      wristPid.setReference(WRIST_POSITION_INTAKING, ControlType.kPosition);
    } else if (outtaking) {
      intakeRollers.set(-0.4);
      shooter.set(0);
      queuer.set(-0.5);
      wristPid.setReference(WRIST_POSITION_OUTTAKING, ControlType.kPosition);
    } else if (shooting) {
      intakeRollers.set(0.4);
      shooter.set(0.60);
      queuer.set(0.5);
      wristPid.setReference(WRIST_POSITION_IDLE, ControlType.kPosition);
    } else {
      shooter.set(0);
      queuer.set(0);
      intakeRollers.set(0);
      wristPid.setReference(WRIST_POSITION_IDLE, ControlType.kPosition);
    }
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Queuer/current", queuer.getOutputCurrent());
    SmartDashboard.putNumber("wrist/angle", wristEncoder.getPosition() * 360 / GEARING);
  }
}
