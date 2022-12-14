package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
  }

  public void setState(SwerveModuleState state) {

  }

  public void log() {

  }
}
