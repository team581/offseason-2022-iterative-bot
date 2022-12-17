// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Shooter {
    private static final double SHOOTER_SHOOTING_VELOCITY = 2000 / 60.0 * 360.0;
    private static final double SHOOTING_IDLE = 800 / 60.0 * 360.0;
    private final SparkMaxPIDController pid;
    private final RelativeEncoder encoder;

    public Shooter(CANSparkMax motor) {
        pid = motor.getPIDController();
        encoder = motor.getEncoder();

        motor.setSmartCurrentLimit(45);

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

        pid.setP(0.005);
        pid.setI(0);
        pid.setD(1);
        pid.setIZone(0);
        pid.setFF(0.00023);
        pid.setOutputRange(0, 1);
    }

    public void log() {
        SmartDashboard.putNumber("Shooter/Velocity", getVelocity());
    }

    public void stop() {
        pid.setReference(SHOOTING_IDLE * 60.0 / 360.0, ControlType.kVelocity);
    }

    public void start() {
        pid.setReference(SHOOTER_SHOOTING_VELOCITY * 60.0 / 360.0, ControlType.kVelocity);
    }

    public boolean isReadyToShoot() {
        double error = getVelocity() - SHOOTER_SHOOTING_VELOCITY;

        return Math.abs(error) <= 200;
    }

    private double getVelocity() {
        return encoder.getVelocity() / 60.0 * 360.0;
    }
}
