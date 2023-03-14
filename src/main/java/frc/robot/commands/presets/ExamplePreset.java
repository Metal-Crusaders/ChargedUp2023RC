//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot.commands.presets;
//
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Pivot;
//
///** An example command that uses an example subsystem. */
//public class ExamplePreset extends CommandBase {
//
//  Pivot pivot;
//  Elevator elevator;
//
//  boolean elevatorIntent;
//  double pivotDegrees;
//
//  int DEADBAND = 5;
//
//  double kP = 0.1; // TODO FIX
//
//  double error, speed;
//
//  double PIVOT_FULL_POWER = 0.2; // TODO test this at some point
//  double ELEVATOR_FULL_POWER = 0.5;
//
//  public ExamplePreset(Pivot pivot, Elevator elevator, boolean elevatorIntent, double pivotDegrees) {
//    this.pivot = pivot;
//    this.elevator = elevator;
//    this.elevatorIntent = elevatorIntent;
//    this.pivotDegrees = pivotDegrees;
//
//    addRequirements(pivot);
//    addRequirements(elevator);
//  }
//
//  @Override
//  public void initialize() {
//  }
//
//  @Override
//  public void execute() {
//    if (elevatorIntent && !(elevator.upperLimitTriggered())) {
//      elevator.set(ELEVATOR_FULL_POWER);
//    } else if ((Math.abs(pivot.getEncoderTicks() - pivotDegrees) <= DEADBAND)) {
//      elevator.stop();
//
//      error = pivot.getEncoderTicks() - pivotDegrees;
//      speed = error * kP * PIVOT_FULL_POWER;
//
//      pivot.set(speed);
//    } else if (!elevatorIntent && !(elevator.lowerLimitTriggered())) {
//      pivot.stop();
//      elevator.set(-ELEVATOR_FULL_POWER);
//    }
//  }
//
//  @Override
//  public void end(boolean interrupted) {
//    pivot.stop();
//    elevator.stop();
//  }
//
//  @Override
//  public boolean isFinished() {
//    return (
//            ((elevatorIntent && elevator.upperLimitTriggered()) ||
//            (!elevatorIntent && elevator.lowerLimitTriggered())) &&
//            ((pivot.getEncoderTicks() < pivotDegrees + DEADBAND) &&
//             (pivot.getEncoderTicks() > pivotDegrees - DEADBAND))
//    );
//  }
//}
