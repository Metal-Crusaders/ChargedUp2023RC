// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.motor.MySparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    MySparkMax motor;

    private final double LOWER_BOUND = 1000, UPPER_BOUND = 15500; // TODO figure out elevator upper bounds

    public Elevator(MySparkMax motor) {
        this.motor = motor;
    }

    // basic motor methods
    public void set(double speed) {
        this.motor.set(speed);
    }

    public void stop() {
        this.motor.set(0);
    }

    public MySparkMax getMotor() {
        return this.motor;
    }

    public void resetEncoder() {
        this.motor.resetEncoder();
    }

    public double getEncoderTicks() {
        return this.motor.getDistance();
    }


    public boolean upperLimitTriggered() {
        return this.getEncoderTicks() >= UPPER_BOUND;
    }

    public boolean lowerLimitTriggered() {
        return this.getEncoderTicks() <= LOWER_BOUND;
    }
}
