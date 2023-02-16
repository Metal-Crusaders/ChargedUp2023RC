// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.tankdrive.RawTankTeleop;
import frc.robot.commands.pivot.RawPivotTeleop;
import frc.robot.motor.MySparkMax;
import frc.robot.motor.MyVictorSPX;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Pivot;

public class RobotContainer {

  // Motors
  public MySparkMax leftFront, leftRear, rightFront, rightRear;
  public MySparkMax leftPivot, rightPivot;

  // Sensors
  public AHRS gyro;

  // Subsystems
  public TankDrive drive;
  public Pivot pivot;

  // OI + Buttons
  public OI oi;

  // Commands
  public RawTankTeleop tankTeleop;
  public RawPivotTeleop pivotTeleop;

  // Auto Commands

  SendableChooser<Command> chooser;


  public RobotContainer() {

    // Motors
    leftFront = new MySparkMax(RobotMap.LEFT_FRONT, true, RobotMap.LEFT_DT_INVERTED);
    leftRear = new MySparkMax(RobotMap.LEFT_REAR, true, RobotMap.LEFT_DT_INVERTED);
    rightFront = new MySparkMax(RobotMap.RIGHT_FRONT, true, !RobotMap.LEFT_DT_INVERTED);
    rightRear = new MySparkMax(RobotMap.RIGHT_REAR, true, !RobotMap.LEFT_DT_INVERTED);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    leftPivot = new MySparkMax(RobotMap.LEFT_PIVOT, true, RobotMap.LEFT_PIV_INVERTED);
    rightPivot = new MySparkMax(RobotMap.RIGHT_PIVOT, true, !RobotMap.LEFT_PIV_INVERTED);

    rightPivot.follow(leftPivot, !RobotMap.LEFT_PIV_INVERTED);

    // Sensors
    gyro = new AHRS(SPI.Port.kMXP);

    // Subsystems
    drive = new TankDrive(leftFront, rightFront, gyro);
    pivot = new Pivot(leftPivot);

    // OI + Buttons
    oi = new OI();

    // Commands
    tankTeleop = new RawTankTeleop(drive, oi::getDriverXboxLeftTrigger, oi::getDriverXboxRightTrigger, oi::getDriverXboxLeftX);
    pivotTeleop = new RawPivotTeleop(pivot, oi::getOperatorXboxLeftY);

    // Auto Commands

    // Sendable Chooser:
    chooser = new SendableChooser<>();
    SmartDashboard.putData(chooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}