package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private final TalonFX steerMotor;
  private final TalonFX driveMotor;
  private final CANCoder canCoder;
  private final Rotation2d angleOffset;
  private final String label;

  public SwerveModule(TalonFX steerMotor, TalonFX driveMotor,
      CANCoder canCoder, Rotation2d angleOffset, String label) {
    this.steerMotor = steerMotor;
    this.driveMotor = driveMotor;
    this.canCoder = canCoder;
    this.angleOffset = angleOffset;
    this.label = label;
    resetSteerMotorAngle();

    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    driveMotor.config_kP(0, 0);
    driveMotor.config_kI(0, 0);
    driveMotor.config_kD(0, 0);
    driveMotor.config_kF(0, 0);
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));

    steerMotor.config_kP(0, 0.67);
    steerMotor.config_kI(0, 0);
    steerMotor.config_kD(0, 0.17);
    steerMotor.config_kF(0, 0);
    steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
  }

  public void setState(SwerveModuleState state) {
    setDriveMotorVelocity(state.speedMetersPerSecond);
    setSteerMotorAngle(state.angle);
  }

  public void log() {
    SmartDashboard.putNumber("Swerve/" + label + "/DriveMotorVelocity", getDriveMotorVelocity());
    SmartDashboard.putNumber("Swerve/" + label + "/SteerMotorAngle", getSteerMotorAngle().getDegrees());
    SmartDashboard.putNumber("Swerve/" + label + "/CancoderLabel", getCancoderAngle().getDegrees());
  }

  private Rotation2d getCancoderAngle() {
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition()).minus(angleOffset);
  }

  private Rotation2d getSteerMotorAngle() {
    return Rotation2d.fromDegrees(steerMotor.getSelectedSensorPosition() / 12.8 / 2048.0);
  }

  private double getDriveMotorVelocity() {
    return driveMotor.getSelectedSensorVelocity() * 10.0 / 2048.0 / 10.0;
  }

  private void setDriveMotorVelocity(double percentage) {
    driveMotor.set(TalonFXControlMode.PercentOutput, percentage);
  }

  private void setSteerMotorAngle(Rotation2d angle) {
    steerMotor.set(TalonFXControlMode.Position, angle.getDegrees() * 12.8 * 2048.0);
  }

  private void resetSteerMotorAngle() {
    Rotation2d canCoderAngle = getCancoderAngle();
    steerMotor.setSelectedSensorPosition(canCoderAngle.getDegrees() * 12.8 * 2048.0);
  }
}
