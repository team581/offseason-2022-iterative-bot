// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

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

  CANSparkMax wrist = new CANSparkMax(16, MotorType.kBrushless);
  XboxController controller = new XboxController(0);
  SparkMaxPIDController wristPID;

  CANSparkMax intakeRollers = new CANSparkMax(15, MotorType.kBrushless);

  CANSparkMax shooter = new CANSparkMax(18, MotorType.kBrushless);

  CANSparkMax queuer = new CANSparkMax(17, MotorType.kBrushless);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    wristPID = wrist.getPIDController();
    wristPID.setP(0.1);
    wristPID.setI(0.1);
    wristPID.setD(0.1);
  }





  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {



    double wristSpeed = -controller.getLeftY();

    wrist.set(wristSpeed / 3);

    boolean intaking = controller.getLeftTriggerAxis() > 0.5;
    boolean shooting = controller.getRightTriggerAxis() > 0.5;
    boolean outtaking = controller.getLeftBumper();
    if (intaking) {
      intakeRollers.set(0.4);
      queuer.set(0);
      shooter.set(0);
    } else if (shooting) {
      shooter.set(0.6);
      queuer.set(0.5);
      intakeRollers.set(0);
    } else if (outtaking) {
      intakeRollers.set(-0.4);
      queuer.set(-0.5);
      shooter.set(0);
    } else {
      shooter.set(0);
      intakeRollers.set(0);
      queuer.set(0);
    }
  }


}
