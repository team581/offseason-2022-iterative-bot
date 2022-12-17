// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Wrist {
    /** 64:1 gearing reduction, 2:1 chain reduction. */
    public static final double WRIST_GEARING = 60 * 2;
    public static final double WRIST_POSITION_OUTTAKING = 60.0 / 360.0 * WRIST_GEARING;
    public static final double WRIST_POSITION_INTAKING = 40.0 / 360.0 * WRIST_GEARING;
    public static final double WRIST_POSITION_IDLE = 110.0 / 360.0 * WRIST_GEARING;

    public final CANSparkMax wrist;
    public final SparkMaxPIDController pid;
    public final RelativeEncoder encoder;

    public Wrist(CANSparkMax wrist) {
        this.wrist = wrist;
        this.pid = wrist.getPIDController();
        this.encoder = wrist.getEncoder();

        pid.setP(0.3);
        pid.setI(0);
        pid.setD(0);
        pid.setIZone(0);
        pid.setFF(0);
        pid.setOutputRange(-1, 1);
    }

    public void log() {
        SmartDashboard.putNumber("wrist/angle", encoder.getPosition() * 360.0 / WRIST_GEARING);
    }

    public void GoToIntakePosition() {
        pid.setReference(WRIST_POSITION_INTAKING, ControlType.kPosition);
    }

    public void GoToOuttakePosition() {
        pid.setReference(WRIST_POSITION_OUTTAKING, ControlType.kPosition);
    }

    public void GoToIdlePosition() {
        pid.setReference(WRIST_POSITION_IDLE, ControlType.kPosition);
    }
}
