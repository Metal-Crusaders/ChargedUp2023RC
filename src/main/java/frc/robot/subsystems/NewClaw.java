// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewClaw extends SubsystemBase {

    VictorSP roller;
    VictorSP wrist;
    Encoder wristEncoder;

    public static final double LOWER_BOUND = -600, UPPER_BOUND = 1e9;

    public static final double SUCK_POWER = 0.25, SPIT_POWER = 0.65;

    public NewClaw(VictorSP roller, VictorSP wrist, Encoder wristEncoder) {
        this.roller = roller;
        this.wrist = wrist;
        this.wristEncoder = wristEncoder;
    }

    public void setRollers(boolean on, boolean off) {
        if (on == off) {
            this.roller.set(0);
        } else if (off) {
            this.roller.set(SPIT_POWER);
        } else {
            this.roller.set(SUCK_POWER);
        }
    }


    public boolean rollersOn() {
        return (this.roller.get() != 0);
    }

    // wrist commands
    public void setWrist(double speed) {
        if (speed < 0 && getWristTicks() > UPPER_BOUND || speed > 0 && getWristTicks() < LOWER_BOUND) {
            speed = 0;
        }
        wrist.set(speed);
    }

    public void rawRollerControl(double speed) {
        this.roller.set(speed);
    }

    // encoder stuff
    public double getWristTicks() {
        return wristEncoder.getDistance();
    }

    public void resetWristEncoder() {
        wristEncoder.reset();
    }
}
