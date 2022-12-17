// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class IntakeRollers {
    private CANSparkMax intakeRollers;

    public IntakeRollers(CANSparkMax intakeRollers) {
        this.intakeRollers = intakeRollers;
    }

    public void intake() {
        intakeRollers.set(0.4);
    }

    public void outtake() {
        intakeRollers.set(-0.4);
    }

    public void stop() {
        intakeRollers.set(0);
    }

    public void setInverted() {
        return;
    }
}
