// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final CANSparkMax wrist = new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax intakeRollers = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax queuer = new CANSparkMax(17, MotorType.kBrushless);
  private final CANSparkMax shooter = new CANSparkMax(18, MotorType.kBrushless);
  private final XboxController controller = new XboxController(0);
 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Boolean shooterandqueuer = controller.getXButton()
    boolean shooterandqueuer = controller.getRightTriggerAxis() > 0.5;
   
    if (shooterandqueuer) {
      shooter.set(0.4);
      queuer.set(0.5);
    } else {
      shooter.set(0);
      queuer.set(0);
    }
  }

}
