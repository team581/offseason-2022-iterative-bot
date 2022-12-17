// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Superstructure {
    private final Wrist wrist;
    private final Shooter shooter;
    private final IntakeRollers intakeRollers;
    private final Queuer queuer;

    public Superstructure(Shooter shooter, Wrist wrist, IntakeRollers intakeRollers, Queuer queuer) {
        this.shooter = shooter;
        this.wrist = wrist;
        this.intakeRollers = intakeRollers;
        this.queuer = queuer;
    }

    public void intaking() {
        intakeRollers.intake();
        shooter.stop();
        wrist.GoToIntakePosition();
        if (queuer.sensor()) {
            queuer.stop();
        } else {
            queuer.toShooter();
        }
    }

    public void outtaking() {
        intakeRollers.outtake();
        shooter.stop();
        queuer.toOuttaker();
        wrist.GoToOuttakePosition();
    }

    public void shooting() {
        shooter.start();
        wrist.GoToIntakePosition();
        if (shooter.isReadyToShoot()) {
            intakeRollers.intake();
            queuer.toShooter();
        } else {
            intakeRollers.stop();
            queuer.stop();
        }
    }

    public void nothingHappening() {
        shooter.stop();
        queuer.stop();
        intakeRollers.stop();
        wrist.GoToIdlePosition();
    }
}
