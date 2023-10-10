// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

/** An example command that uses an example subsystem. */
public class ElevatorPreset extends CommandBase {

  Elevator elevator;

  boolean elevatorIntent;

  double ELEVATOR_FULL_POWER = 0.5;

  public ElevatorPreset(Elevator elevator, boolean elevatorIntent) {
    this.elevator = elevator;
    this.elevatorIntent = elevatorIntent;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (elevatorIntent && !(elevator.upperLimitTriggered())) {
      elevator.set(ELEVATOR_FULL_POWER);
    } else {
      elevator.set(-ELEVATOR_FULL_POWER);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return (
        (elevatorIntent && elevator.upperLimitTriggered()) ||
        (!elevatorIntent && elevator.lowerLimitTriggered())
    );
  }
}