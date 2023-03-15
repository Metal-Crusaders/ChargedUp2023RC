// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/** An example command that uses an example subsystem. */
public class ExamplePreset extends CommandBase {

  Pivot pivot;
  Elevator elevator;
  Claw claw;

  boolean elevatorIntent;
  double pivotTicks;
  double clawTicks;

  int DEADBAND_PIVOT = 5;
  int DEADBAND_CLAW = 5;

  double kPPivot = 0.1; // TODO TEST
  double kPClaw = 0.1;

  double pivotSpeed, clawSpeed;

  double PIVOT_FULL_POWER = 0.25;
  double ELEVATOR_FULL_POWER = 0.5;
  double WRIST_FULL_POWER = 0.1; // TODO VERIFY THIS IS GOOD

  public ExamplePreset(Pivot pivot, Elevator elevator, Claw claw, boolean elevatorIntent, double pivotTicks, double clawTicks) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.elevatorIntent = elevatorIntent;
    this.pivotTicks = pivotTicks;
    this.clawTicks = clawTicks;

    addRequirements(pivot);
    addRequirements(elevator);
    addRequirements(claw);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (elevatorIntent && !(elevator.upperLimitTriggered())) {
      elevator.set(ELEVATOR_FULL_POWER);
    } else if (Math.abs(pivot.getEncoderTicks() - pivotTicks) <= DEADBAND_PIVOT) {
      elevator.stop();

      pivotSpeed = (pivot.getEncoderTicks() - pivotTicks) * kPPivot * PIVOT_FULL_POWER;

      pivot.set(pivotSpeed);
    } else if (!elevatorIntent && !(elevator.lowerLimitTriggered())) {
      pivot.stop();
      elevator.set(-ELEVATOR_FULL_POWER);
    } else if (Math.abs(claw.getWristTicks() - clawTicks) <= DEADBAND_CLAW) {
      // once everything else is done, do claw stuff
      clawSpeed = (claw.getWristTicks() - clawTicks) * kPClaw * WRIST_FULL_POWER;
    }
  }

  @Override
  public void end(boolean interrupted) {
    pivot.stop();
    elevator.stop();
    claw.setWrist(0);
  }

  @Override
  public boolean isFinished() {
    return (
            ((elevatorIntent && elevator.upperLimitTriggered()) ||
            (!elevatorIntent && elevator.lowerLimitTriggered())) &&
            (Math.abs(pivot.getEncoderTicks() - pivotTicks) <= DEADBAND_PIVOT) &&
            (Math.abs(claw.getWristTicks() - clawTicks) <= DEADBAND_CLAW)
    );
  }
}
