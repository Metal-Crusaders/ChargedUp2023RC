// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.motor.MyVictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final MyVictorSPX pivot, extender;

  public Arm(MyVictorSPX pivot, MyVictorSPX extender) {
    this.pivot = pivot;
    this.extender = extender;

    pivot.brake();
    extender.brake();
  }

  // PIVOT MOTOR METHODS
  public MyVictorSPX getPivot() {
    return pivot;
  }

  public void setPivot(double speed) {
    pivot.set(speed);
  }

  public void stopPivot() {
    pivot.stop();
  }

  public double getPivotDistance() {
    return pivot.getDistance();
  }

  // EXTENDER MOTOR METHODS:
  public MyVictorSPX getExtender() {
    return extender;
  }

  public void setExtender(double speed) {
    extender.set(speed);
  }

  public void stopExtender() {
    extender.stop();
  }

  public double getExtenderDistance() {
    return extender.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
