// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

/** An example command that uses an example subsystem. */
public class ClawPreset extends CommandBase {

  Claw claw;

  double clawTicks;

  int DEADBAND_CLAW = 5;

  double clawSpeed;
  boolean clawOpen;

  double WRIST_FULL_POWER = 0.1;

  public ClawPreset(Claw claw, double clawTicks, boolean clawOpen) {
    this.claw = claw;
    this.clawTicks = clawTicks;
    this.clawOpen = clawOpen;

    addRequirements(claw);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      // once everything else is done, do claw stuff
      if (!clawOpen) {
        claw.set(clawOpen);
      } else if (Math.abs(claw.getWristTicks() - clawTicks) >= DEADBAND_CLAW) {
        clawSpeed = (claw.getWristTicks() - clawTicks) / (Math.abs(claw.getWristTicks() - clawTicks)) * WRIST_FULL_POWER;
        claw.setWrist(clawSpeed);
      }
      if (clawOpen) {
        claw.set(clawOpen);
      }
  }

  @Override
  public void end(boolean interrupted) {
    claw.setWrist(0);
  }

  @Override
  public boolean isFinished() {
    return (
        (Math.abs(claw.getWristTicks() - clawTicks) <= DEADBAND_CLAW) &&
        ((clawOpen && claw.isOpen()) || !(clawOpen && !claw.isOpen()))
    );
  }
}