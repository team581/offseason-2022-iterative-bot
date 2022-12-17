// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class Queuer {
    public CANSparkMax motor;
    public DigitalInput sensor = new DigitalInput(0);

    public Queuer(CANSparkMax queuer) {
        this.motor = queuer;
    }

    public void toShooter() {
        motor.set(0.4);
    }

    public void toOuttaker() {
        motor.set(-0.5);
    }

    public void stop() {
        motor.set(0);
    }

    public boolean hasBall() {
        return sensor.get(); 
    }
}
