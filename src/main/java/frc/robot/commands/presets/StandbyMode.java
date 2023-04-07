// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/** An example command that uses an example subsystem. */
public class StandbyMode extends CommandBase {

  Pivot pivot;
  Elevator elevator;
  Claw claw;

  public StandbyMode(Pivot pivot, Elevator elevator, Claw claw) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.claw = claw;

    addRequirements(pivot);
    addRequirements(elevator);
    addRequirements(claw);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    if (pivot.getEncoderTicks() >= Pivot.STRAIGHT_UP) {
      pivot.set(-0.1);
      claw.setWrist(0.1);
    } else {
      pivot.set(0.1);
      claw.setWrist(-0.1);
    }

    elevator.set(-0.1);
  }

  @Override
  public void end(boolean interrupted) {
    pivot.stop();
    elevator.stop();
    claw.setWrist(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
