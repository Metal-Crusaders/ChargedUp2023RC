// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

  public static final double LOWER_BOUND = -1e9;
  public static final double UPPER_BOUND = 100000;
  public static final double STRAIGHT_UP = 952.750000;
  private final VictorSP pivotL, pivotR;
  private final Encoder encoder;

  public Pivot(VictorSP pivotL, VictorSP pivotR, Encoder encoder) {
    this.pivotL = pivotL;
    this.pivotR = pivotR;
    this.encoder = encoder;

    resetEncoder();
  }

  // PIVOT MOTOR METHODS
  public VictorSP getLeftMotor() {
    return pivotL;
  }

  public VictorSP getRightMotor() {
    return pivotR;
  }

  public void set(double speed) {
    if (speed > 0 && getEncoderTicks() > UPPER_BOUND || speed < 0 && getEncoderTicks() < LOWER_BOUND) {
      speed = 0;
    }
    pivotL.set(speed);
    pivotR.set(speed);
  }

  public void stop() {
    this.set(0);
  }

  public double getEncoderTicks() {
    return encoder.getDistance();
  }

  public void resetEncoder() {
    encoder.reset();
  }

}
