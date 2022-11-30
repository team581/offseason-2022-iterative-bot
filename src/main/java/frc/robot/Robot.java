// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private static final double GEARING = 64;
  private static final double WRIST_POSITION_UP = 0 * GEARING; 
  private static final double WRIST_POSITION_DOWN = 0 * GEARING;
  CANSparkMax intakeRollers = new CANSparkMax(15, MotorType.kBrushless);
  CANSparkMax wrist = new CANSparkMax(16, MotorType.kBrushless);
  CANSparkMax shooter = new CANSparkMax(17, MotorType.kBrushless);
  CANSparkMax queuer = new CANSparkMax(18, MotorType.kBrushless);
  XboxController controller = new XboxController(0);
  SparkMaxPIDController wristPid; 
  SparkMaxRelativeEncoder wristEncoder;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    wristPid = wrist.getPIDController();
    wristPid.setP(0);
    wristPid.setI(0);
    wristPid.setD(0);
    wristPid.setIZone(0);
    wristPid.setFF(0);
    wristPid.setOutputRange(-0.5, 0.5);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean rolling = controller.getLeftTriggerAxis() > 0.5;
    boolean backwardsRolling = controller.getLeftBumper();
    if (rolling) {
      intakeRollers.set(0.4);
      wristPid.setReference(WRIST_POSITION_UP, ControlType.kPosition);
    } else if (backwardsRolling) {
      intakeRollers.set(-0.4);
      queuer.set(-0.5);
      wristPid.setReference(WRIST_POSITION_UP, ControlType.kPosition);
    } else {
      intakeRollers.set(0);
      queuer.set(0);
      wristPid.setReference(WRIST_POSITION_DOWN, ControlType.kPosition);
    }

    boolean shooting = controller.getRightTriggerAxis() > 0.5;
    if (shooting) {
      shooter.set(0.60);
      queuer.set(0.5);
    } else {
      shooter.set(0);
      queuer.set(0);
    }
  }

  @Override
  public void robotPeriodic() {
       SmartDashboard.putNumber("wrist/angle", wristEncoder.getPosition() / GEARING);
  }

}
