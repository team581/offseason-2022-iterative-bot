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

CANSparkMax wrist = new CANSparkMax(16, MotorType.kBrushless);
XboxController controller= new XboxController(0);

CANSparkMax intakeRollers = new CANSparkMax(15, MotorType.kBrushless);

CANSparkMax shooter = new CANSparkMax(17,MotorType.kBrushless);

CANSparkMax queuer = new CANSparkMax(18,MotorType.kBrushless);



/**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   @Override
  public void robotInit() {

  }



  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    double wristSpeed = controller.getLeftY();

    wrist.set(wristSpeed/3);

    double queuerSpeed = controller.getLeftTriggerAxis();
      if (queuerSpeed > 0.1)
      intakeRollers.set(-0.4);
      queuer.set(-0.5);

   else intakeRollers.set(0);
    queuer.set(0);

    double shooterSpeed = controller.getRightTriggerAxis();
    if(shooterSpeed > 0.1)
    shooter.set(0.6);
    queuer.set(0.5);

    boolean intakeRollersSpeed = controller.getLeftBumper();
     if(intakeRollersSpeed)
     intakeRollers.set(0.4);
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
