// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    DoubleSolenoid piston;
    VictorSP roller1, roller2;
    VictorSP wrist;
    Encoder wristEncoder;

    public static final double LOWER_BOUND = -600, UPPER_BOUND = 0, PICKUP = -500;

    public static final double SUCK_POWER = 0.25, SPIT_POWER = 0.65;

    public Claw(DoubleSolenoid piston, VictorSP roller1, VictorSP roller2, VictorSP wrist, Encoder wristEncoder) {
        this.piston = piston;
        this.roller1 = roller1;
        this.roller2 = roller2;
        this.wrist = wrist;
        this.wristEncoder = wristEncoder;

        this.roller1.setSafetyEnabled(true);
        this.roller2.setSafetyEnabled(true);
    }

    public void set(boolean open) {
        if (open) {
            piston.set(DoubleSolenoid.Value.kForward);
        } else {
            piston.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public boolean isOpen() {
        return (piston.get() == DoubleSolenoid.Value.kForward);
    }

    public void setRollers(boolean on, boolean off) {
        if (on == off) {
            this.roller1.set(0);
            this.roller2.set(0);
        } else if (off) {
            this.roller1.set(SPIT_POWER);
            this.roller2.set(-SPIT_POWER);
        } else {
            this.roller1.set(-SUCK_POWER);
            this.roller2.set(SUCK_POWER);
        }
    }


    public boolean rollersOn() {
        return (this.roller1.get() != 0 || this.roller2.get() != 0);
    }

    // wrist commands
    public void setWrist(double speed) {
        if (speed > 0 && getWristTicks() > UPPER_BOUND || speed < 0 && getWristTicks() < LOWER_BOUND) {
            speed = 0;
        }
        wrist.set(speed);
    }

    public void rawRollerControl(double speed) {
        this.roller1.set(speed);
        this.roller2.set(-speed);
    }

    // encoder stuff
    public double getWristTicks() {
        return wristEncoder.getDistance();
    }

    public void resetWristEncoder() {
        wristEncoder.reset();
    }
}
