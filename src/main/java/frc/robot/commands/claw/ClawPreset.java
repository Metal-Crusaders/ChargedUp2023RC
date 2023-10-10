// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

/** An example command that uses an example subsystem. */
public class ClawPreset extends CommandBase {

  Claw claw;

  double clawTicks;

  double DEADBAND_CLAW = 5;

  double clawSpeed;

  double WRIST_FULL_POWER = 0.7;

  public ClawPreset(Claw claw, double clawTicks) {
    this.claw = claw;
    this.clawTicks = clawTicks;

    addRequirements(claw);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      // once everything else is done, do claw stuff
      clawSpeed = (claw.getWristTicks() - clawTicks) / (Math.abs(claw.getWristTicks() - clawTicks)) * WRIST_FULL_POWER;
      SmartDashboard.putNumber("Claw Speed Defined in Claw Preset", clawSpeed);
      claw.setWrist(clawSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    claw.setWrist(0);
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("donzo", (
      (Math.abs(claw.getWristTicks() - clawTicks) <= DEADBAND_CLAW)
    ));
    return (
        (Math.abs(claw.getWristTicks() - clawTicks) <= DEADBAND_CLAW)
    );
  }
}