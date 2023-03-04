// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    DoubleSolenoid piston;
    VictorSP roller1, roller2;

    public final static double ROLLER_SPEED = 0.5;

    public Claw(DoubleSolenoid piston, VictorSP roller1, VictorSP roller2) {
        this.piston = piston;
        this.roller1 = roller1;
        this.roller2 = roller2;
    }

    public void set(boolean open) {
        if (open) {
            piston.set(DoubleSolenoid.Value.kForward);
        } else {
            piston.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void pistonOff() {
        piston.set(DoubleSolenoid.Value.kOff);
    }

    public boolean isOpen() {
        return (piston.get() == DoubleSolenoid.Value.kForward);
    }

    public void setRollers(boolean on) {
        if (on) {
            this.roller1.set(ROLLER_SPEED);
            this.roller2.set(ROLLER_SPEED);
        } else {
            this.roller1.set(0);
            this.roller2.set(0);
        }

    }

    public boolean rollersOn() {
        return (this.roller1.get() != 0 || this.roller2.get() != 0);
    }
}
