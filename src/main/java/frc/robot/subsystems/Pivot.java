// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.motor.MySparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

  public static final int TICKS_PER_ROTATION = 420; // TODO change this
  public static final int PIVOT_GEAR_RATIO = 27;

  private final MySparkMax pivot;

  public Pivot(MySparkMax pivot) {
    this.pivot = pivot;

    pivot.brake();
  }

  // PIVOT MOTOR METHODS
  public MySparkMax getMotor() {
    return pivot;
  }

  public void set(double speed) {
    pivot.set(speed);
  }

  public void stop() {
    pivot.stop();
  }

  public double getEncoderTicks() {
    return pivot.getDistance();
  }
  public double getSpeed() {
    return pivot.getSpeed(TICKS_PER_ROTATION);
  }

  public void resetEncoder() {
    pivot.resetEncoder();
  }
  

  @Override
  public void periodic() {
  }

}
