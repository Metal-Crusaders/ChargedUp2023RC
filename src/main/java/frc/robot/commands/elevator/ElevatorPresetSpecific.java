// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

/** An example command that uses an example subsystem. */
public class ElevatorPresetSpecific extends CommandBase {

  Elevator elevator;

  double encoderTicks;

  double ELEVATOR_FULL_POWER = 0.3;
  private double DEADBAND = 100;

  public ElevatorPresetSpecific(Elevator elevator, double encoderTicks) {
    this.elevator = elevator;
    this.encoderTicks = encoderTicks;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double sign = (encoderTicks - elevator.getEncoderTicks()) / Math.abs(encoderTicks - elevator.getEncoderTicks());
    elevator.set(ELEVATOR_FULL_POWER * sign);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return (
        (Math.abs(encoderTicks - elevator.getEncoderTicks()) < DEADBAND)
    );
  }
}