// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    VictorSP motor1, motor2;
    Encoder encoder;

    public Elevator(VictorSP motor1, VictorSP motor2, Encoder encoder) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.encoder = encoder;

        setDistancePerPulse(420);
    }

    // basic motor methods
    public void set(double speed) {
        this.motor1.set(speed);
        this.motor2.set(speed);
    }

    public void stop() {
        this.motor1.set(0);
        this.motor2.set(0);
    }

    public VictorSP getMotor1() {
        return this.motor1;
    }

    public VictorSP getMotor2() {
        return this.motor2;
    }

    // basic encoder methods
    public void resetEncoder() {
        this.encoder.reset();
    }

    public void setDistancePerPulse(double dpp) {
        this.encoder.setDistancePerPulse(dpp);
    }

    public double getDistance() {
        return this.encoder.getDistance();
    }

    public Encoder getEncoder() {
        return this.encoder;
    }
}
