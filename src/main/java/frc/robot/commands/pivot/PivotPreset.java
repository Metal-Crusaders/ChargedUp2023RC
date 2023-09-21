// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

/** An example command that uses an example subsystem. */
public class PivotPreset extends CommandBase {

  Pivot pivot;

  double pivotTicks;

  double DEADBAND_PIVOT = 5;

  double pivotSpeed;

  double PIVOT_FULL_POWER = 0.3;

  public PivotPreset(Pivot pivot, double pivotTicks) {
    this.pivot = pivot;
    this.pivotTicks = pivotTicks;

    addRequirements(pivot);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    pivotSpeed = (pivotTicks - pivot.getEncoderTicks()) / (Math.abs(pivotTicks - pivot.getEncoderTicks())) * PIVOT_FULL_POWER; // (pivot.getEncoderTicks() - pivotTicks) * kPPivot * 

    pivot.set(pivotSpeed);
    // SmartDashboard.putBoolean("second part", (Math.abs(pivot.getEncoderTicks() - pivotTicks) <= DEADBAND_PIVOT));
  }

  @Override
  public void end(boolean interrupted) {
    pivot.stop();
  }

  @Override
  public boolean isFinished() {
    return (
        (Math.abs(pivot.getEncoderTicks() - pivotTicks) <= DEADBAND_PIVOT)
    );
  }
}
