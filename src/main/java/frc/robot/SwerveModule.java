// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveModule {
    private static final double ticksPerRotation = 12.8 * 2048;
    public final TalonFX drive;
    public final TalonFX steering;
    public final String name;

    public SwerveModule(int driveID, int steeringID, String name) {
        steering = new TalonFX(steeringID);
        drive = new TalonFX(driveID);
        this.name = name;
        steering.config_kP(0, 0.67);
        steering.config_kI(0, 0);
        steering.config_kD(0, 0.17);
        steering.config_kF(0, 0);
        steering.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
        steering.setInverted(false);
    }

    public void setRotation(double value) {
        steering.set(TalonFXControlMode.Position, value * ticksPerRotation);
    }

    public void setAngleAndDrive(double angle, double driving) {
        steering.set(TalonFXControlMode.Position, angle * ticksPerRotation);
        drive.set(TalonFXControlMode.PercentOutput, driving);
    }

    public void log() {
        SmartDashboard.putNumber("Swerve/" + name + "/Position", steering.getSelectedSensorPosition() / ticksPerRotation);
    }
}
