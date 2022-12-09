// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.ControlType;

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

  private final CANSparkMax wrist = new CANSparkMax(16, MotorType.kBrushless);
  private final XboxController controller = new XboxController(0);
  private final SparkMaxPIDController wristPID = wrist.getPIDController();
  private final RelativeEncoder wristEncoder = wrist.getEncoder();
  private final DigitalInput queuerSensor = new DigitalInput(0);
  private final CANSparkMax intakeRollers = new CANSparkMax(15, MotorType.kBrushless);

  private final CANSparkMax shooter = new CANSparkMax(18, MotorType.kBrushless);

  private final CANSparkMax queuer = new CANSparkMax(17, MotorType.kBrushless);

  private static final double WRIST_GEARING = 60.0 * 2.0;
  private static final double WRIST_POSITION_IDLE = 120;
  private static final double WRIST_POSITION_INTAKING = 40;
  private static final double WRIST_POSITION_OUTTAKING = 60;

  @Override
  public void robotInit() {
    wristPID.setP(0.1);
    wristPID.setI(0);
    wristPID.setD(0);
    wristPID.setIZone(0.0);
    wristPID.setOutputRange(-1, 1);
    intakeRollers.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Wrist/Angle", wristEncoder.getPosition() * 360.0 / WRIST_GEARING);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean intaking = controller.getLeftTriggerAxis() > 0.5;
    boolean shooting = controller.getRightTriggerAxis() > 0.5;
    boolean outtaking = controller.getLeftBumper();

    if (intaking) {
      intakeRollers.set(0.4);
      shooter.set(0);
      wristPID.setReference(WRIST_POSITION_INTAKING / 360.0 * WRIST_GEARING, ControlType.kPosition);
      if (queuerSensor.get()) {
        queuer.set(0);
      } else {
        queuer.set(0.2);
      }
    } else if (shooting) {
      shooter.set(0.4);
      queuer.set(0.5);
      intakeRollers.set(0.4);
      wristPID.setReference(WRIST_POSITION_INTAKING / 360.0 * WRIST_GEARING, ControlType.kPosition);
    } else if (outtaking) {
      intakeRollers.set(-0.4);
      queuer.set(-0.5);
      shooter.set(0);
      wristPID.setReference(WRIST_POSITION_OUTTAKING / 360.0 * WRIST_GEARING, ControlType.kPosition);
    } else {
      shooter.set(0);
      intakeRollers.set(0);
      queuer.set(0);
      wristPID.setReference(WRIST_POSITION_IDLE / 360.0 * WRIST_GEARING, ControlType.kPosition);
    }
  }
}
