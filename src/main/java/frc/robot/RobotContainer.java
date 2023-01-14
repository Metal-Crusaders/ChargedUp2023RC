// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.tankdrive.RawTankTeleop;
import frc.robot.motor.MySparkMax;
import frc.robot.subsystems.TankDrive;

public class RobotContainer {

  // Motors
  public MySparkMax leftFront, leftRear, rightFront, rightRear;

  // Subsystems
  public TankDrive drive;

  // OI + Buttons
  public OI oi;

  // Commands
  public RawTankTeleop tankTeleop;


  public RobotContainer() {

    // Motors
    leftFront = new MySparkMax(RobotMap.LEFT_FRONT, true, RobotMap.LEFT_INVERTED);
    leftRear = new MySparkMax(RobotMap.LEFT_REAR, true, RobotMap.LEFT_INVERTED);
    rightFront = new MySparkMax(RobotMap.RIGHT_FRONT, true, !RobotMap.LEFT_INVERTED);
    rightRear = new MySparkMax(RobotMap.RIGHT_REAR, true, !RobotMap.LEFT_INVERTED);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    // Subsystems
    drive = new TankDrive(leftFront, rightFront);

    // OI + Buttons
    oi = new OI();

    // Commands
    tankTeleop = new RawTankTeleop(drive, oi::getDriverXboxLeftTrigger, oi::getDriverXboxRightTrigger, oi::getDriverXboxLeftX);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}