// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    VictorSP motor1, motor2;
    DigitalInput lowerLimit, upperLimit;

    public Elevator(VictorSP motor1, VictorSP motor2, DigitalInput lowerLimit, DigitalInput upperLimit) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
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

    // basic limit switch methods:

    public DigitalInput getUpperLimit() {
        return upperLimit;
    }

    public DigitalInput getLowerLimit() {
        return lowerLimit;
    }

    public boolean upperLimitTriggered() {
//        return upperLimit.get();
        return false;
    }

    public boolean lowerLimitTriggered() {
//        return lowerLimit.get();
        return false;
    }
}
