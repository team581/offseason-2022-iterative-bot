// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveModule {
    //ticksPerRotation is 12.8 (gearing) multiplied by the amount of times the sensor doesn't see itself in one rotation which is 2048
    private static final double ticksPerRotation = 12.8 * 2048;
    public final TalonFX drive;
    public final TalonFX steering;
    public final String name;
    private final CANCoder encoder;
    private final Rotation2d swerveWheelOffset;

    public SwerveModule(int driveID, int steeringID, String name, CANCoder encoder, Rotation2d swerveWheelOffset) {
        steering = new TalonFX(steeringID);
        drive = new TalonFX(driveID);
        this.name = name;
        this.encoder = encoder;
        this.swerveWheelOffset = swerveWheelOffset;
        steering.config_kP(0, 0.67);
        steering.config_kI(0, 0);
        steering.config_kD(0, 0.17);
        steering.config_kF(0, 0);
        steering.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
        steering.setInverted(false);

        resetWheelAngle();
    }

    public void setRotation(double value) {
        steering.set(TalonFXControlMode.Position, value * ticksPerRotation);
    }

    public void setAngleAndDrive(Rotation2d angle, double driving) {
        //divide by 360 bc 360 degrees is 1 rotation and we don't have getRotations() so we need to convert degrees to rotations. 
        steering.set(TalonFXControlMode.Position, angle.getDegrees() * ticksPerRotation / 360);
        drive.set(TalonFXControlMode.PercentOutput, driving);
    }

    public void log() {
        SmartDashboard.putNumber("Swerve/" + name + "/Position", steering.getSelectedSensorPosition() / ticksPerRotation);
        // SmartDashboard.putNumber("Swerve/" + name + "/Speed", drive.get)
    }

    private void resetWheelAngle() {
        Rotation2d actualRotations = getCancoderPosition(); 
        double rotations = actualRotations.getDegrees() / 360;
        double encoderTicks = rotations * ticksPerRotation;
        steering.setSelectedSensorPosition(encoderTicks);
    }

    private Rotation2d getCancoderPosition() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition()).minus(swerveWheelOffset); 
    }
}
