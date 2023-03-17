// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.ChargePanelAuto;
import frc.robot.commands.autonomous.LeaveAndChargeAuto;
import frc.robot.commands.autonomous.tools.DoNothing;
import frc.robot.commands.autonomous.tools.DriveStraightAuto;
import frc.robot.commands.claw.ClawTeleop;
import frc.robot.commands.elevator.RawElevatorTeleop;
import frc.robot.commands.tankdrive.RawTankTeleop;
import frc.robot.commands.pivot.RawPivotTeleop;
import frc.robot.motor.MySparkMax;
import frc.robot.sensors.MyButton;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Pivot;

public class RobotContainer {

  // Motors
  public MySparkMax leftFront, leftRear, rightFront, rightRear;
  public VictorSP leftPivot, rightPivot;
  public VictorSP elevatorMotor1, elevatorMotor2;
  public VictorSP clawRoller1, clawRoller2, clawWrist;

  // Pneumatics
  DoubleSolenoid clawSolenoid;

  // Sensors
  public AHRS gyro;
  public DigitalInput elevatorLower, elevatorUpper;
  public Encoder pivotEncoder;
  public Encoder wristEncoder;
  public AddressableLED leds;
  public AddressableLEDBuffer ledBuf;

  // Subsystems
  public TankDrive drive;
  public Pivot pivot;
  public Elevator elevator;
  public Claw claw;

  // OI + Buttons
  public OI oi;
  public MyButton clawOpenBtn, clawRollerBtn, clawRollerOppBtn;
  public MyButton purpleBtn, yellowBtn;

  // Commands
  public RawTankTeleop tankTeleop;
  public RawPivotTeleop pivotTeleop;
  public RawElevatorTeleop elevatorTeleop;
  public ClawTeleop clawTeleop;

  // Auto Commands
  DoNothing doNothingAuto;
  DriveStraightAuto exitCommunityAuto;
  ChargePanelAuto chargePanelAuto;
  LeaveAndChargeAuto leaveAndCharge;

  SendableChooser<Command> chooser;


  public RobotContainer() {

    // Motors
    leftFront = new MySparkMax(RobotMap.LEFT_FRONT, true, RobotMap.LEFT_DT_INVERTED);
    leftRear = new MySparkMax(RobotMap.LEFT_REAR, true, RobotMap.LEFT_DT_INVERTED);
    rightFront = new MySparkMax(RobotMap.RIGHT_FRONT, true, !RobotMap.LEFT_DT_INVERTED);
    rightRear = new MySparkMax(RobotMap.RIGHT_REAR, true, !RobotMap.LEFT_DT_INVERTED);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    leftPivot = new VictorSP(RobotMap.LEFT_PIVOT);
    rightPivot = new VictorSP(RobotMap.RIGHT_PIVOT);

    leftPivot.setInverted(RobotMap.LEFT_PIV_INVERTED);
    rightPivot.setInverted(!RobotMap.LEFT_PIV_INVERTED);

    elevatorMotor1 = new VictorSP(RobotMap.ELEVATOR_PWM_ID1);
    elevatorMotor1.setInverted(RobotMap.ELEVATOR_REVERSED);
    elevatorMotor2 = new VictorSP(RobotMap.ELEVATOR_PWM_ID2);
    elevatorMotor2.setInverted(!RobotMap.ELEVATOR_REVERSED);

    clawRoller1 = new VictorSP(RobotMap.CLAW_ROLLER1);
    clawRoller1.setInverted(RobotMap.CLAW_REVERSED);
    clawRoller2 = new VictorSP(RobotMap.CLAW_ROLLER2);
    clawRoller2.setInverted(!RobotMap.CLAW_REVERSED);

    clawWrist = new VictorSP(RobotMap.CLAW_WRIST);
//
    // Pneumatics
    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLAW_IN, RobotMap.CLAW_OUT);

    // Sensors
    gyro = new AHRS(SPI.Port.kMXP);
    elevatorLower = new DigitalInput(RobotMap.ELEVATOR_LOWER);
    elevatorUpper = new DigitalInput(RobotMap.ELEVATOR_UPPER);
    pivotEncoder = new Encoder(RobotMap.PIVOT_ENCODER_IN, RobotMap.PIVOT_ENCODER_OUT);
    wristEncoder = new Encoder(RobotMap.WRIST_ENCODER_IN, RobotMap.WRIST_ENCODER_OUT);

    // Subsystems
    drive = new TankDrive(leftFront, rightFront, gyro);
    pivot = new Pivot(leftPivot, rightPivot, pivotEncoder);
    elevator = new Elevator(elevatorMotor1, elevatorMotor2, elevatorLower, elevatorUpper);
    claw = new Claw(clawSolenoid, clawRoller1, clawRoller2, clawWrist, wristEncoder);

    // OI + Buttons
    oi = new OI();
    clawOpenBtn = new MyButton(oi.getOperatorXbox(), OI.XBOX_A);
    clawRollerBtn = new MyButton(oi.getOperatorXbox(), OI.XBOX_X);
    clawRollerOppBtn = new MyButton(oi.getOperatorXbox(), OI.XBOX_Y);
    purpleBtn = new MyButton(oi.getDriverXbox(), OI.XBOX_A);
    yellowBtn = new MyButton(oi.getDriverXbox(), OI.XBOX_B);

    // Commands
    tankTeleop = new RawTankTeleop(
            drive,
            oi::getDriverXboxLeftTrigger, oi::getDriverXboxRightTrigger, oi::getDriverXboxLeftX,
            purpleBtn::isPressed, yellowBtn::isPressed
    );
    pivotTeleop = new RawPivotTeleop(pivot, oi::getOperatorXboxLeftY);
    elevatorTeleop = new RawElevatorTeleop(elevator, oi::getOperatorXboxRightTrigger, oi::getOperatorXboxLeftTrigger);
    clawTeleop = new ClawTeleop(
            claw,
            clawOpenBtn::isPressed, clawRollerBtn::getRaw, clawRollerOppBtn::getRaw, oi::getOperatorXboxRightY
    );

    // presets here

    // Auto Commands
    doNothingAuto = new DoNothing();
    exitCommunityAuto = new DriveStraightAuto(drive, 30000); // TODO need to verify that this is OK
    chargePanelAuto = new ChargePanelAuto(drive, false);
    leaveAndCharge = new LeaveAndChargeAuto(drive);

    // Sendable Chooser:
    chooser = new SendableChooser<>();
    SmartDashboard.putData(chooser);

    chooser.addOption("Do Nothing Auto", doNothingAuto);
    chooser.addOption("Exit Community Auto", exitCommunityAuto);
    chooser.addOption("Charge Panel Middle Auto", chargePanelAuto);
    chooser.addOption("Leave Community + Charge Auto", leaveAndCharge);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}