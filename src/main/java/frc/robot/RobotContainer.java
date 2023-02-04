// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.DriveStraightAuto;
import frc.robot.commands.autonomous.ForwardBackward;
import frc.robot.commands.tankdrive.RawTankTeleop;
import frc.robot.motor.MySparkMax;
import frc.robot.motor.MyVictorSPX;
import frc.robot.subsystems.TankDrive;

public class RobotContainer {

  // Motors
  public MySparkMax leftFront, leftRear, rightFront, rightRear;
  public MyVictorSPX leftPivot, rightPivot, extender;

  // Sensors
  public AHRS gyro;

  // Subsystems
  public TankDrive drive;

  // OI + Buttons
  public OI oi;

  // Commands
  public RawTankTeleop tankTeleop;

  // Auto Commands
  public DriveStraightAuto driveStraightAuto;
  public ForwardBackward forwardBackward;

  SendableChooser<Command> chooser;


  public RobotContainer() {

    // Motors
    leftFront = new MySparkMax(RobotMap.LEFT_FRONT, true, RobotMap.LEFT_DT_INVERTED);
    leftRear = new MySparkMax(RobotMap.LEFT_REAR, true, RobotMap.LEFT_DT_INVERTED);
    rightFront = new MySparkMax(RobotMap.RIGHT_FRONT, true, !RobotMap.LEFT_DT_INVERTED);
    rightRear = new MySparkMax(RobotMap.RIGHT_REAR, true, !RobotMap.LEFT_DT_INVERTED);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    // Sensors
    gyro = new AHRS(SPI.Port.kMXP);

    // Subsystems
    drive = new TankDrive(leftFront, rightFront, gyro);

    // OI + Buttons
    oi = new OI();

    // Commands
    tankTeleop = new RawTankTeleop(drive, oi::getDriverXboxLeftTrigger, oi::getDriverXboxRightTrigger, oi::getDriverXboxLeftX);

    // Auto Commands
    driveStraightAuto = new DriveStraightAuto(drive, -26);
    forwardBackward = new ForwardBackward(drive, 26, 26);

    // Sendable Chooser:
    chooser = new SendableChooser<>();
    chooser.addOption("Drive Test Auto", driveStraightAuto);
    chooser.addOption("Forward-Backward Test", forwardBackward);
    SmartDashboard.putData(chooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}