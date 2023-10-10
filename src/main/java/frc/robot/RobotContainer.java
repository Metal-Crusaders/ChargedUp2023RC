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
import frc.robot.commands.autonomous.*;
import frc.robot.commands.autonomous.tools.HighCubeShot;
import frc.robot.commands.autonomous.tools.MidCubeShot;
import frc.robot.commands.claw.ClawTeleop;
import frc.robot.commands.elevator.RawElevatorTeleop;
import frc.robot.commands.tankdrive.RawTankTeleop;
import frc.robot.commands.pivot.RawPivotTeleop;
import frc.robot.commands.presets.DefaultPreset;
import frc.robot.commands.presets.GroundPreset;
import frc.robot.commands.presets.UpPreset;
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
  public MySparkMax elevatorMotor;
  public VictorSP clawRoller1, clawRoller2, clawWrist;

  // Pneumatics
  DoubleSolenoid clawSolenoid;

  // Sensors
  public AHRS gyro;
  public Encoder pivotEncoder, wristEncoder;
  public AddressableLED leds;
  public AddressableLEDBuffer ledBuf;

  // Subsystems
  public TankDrive drive;
  public Pivot pivot;
  public Elevator elevator;
  public Claw claw;

  // OI + Buttons
  public OI oi;
  public MyButton clawOpenBtn, clawRollerBtn, clawRollerOppBtn, shootMidBtn, shootHighBtn;
  public MyButton sensitivityShifter, purpleBtn, yellowBtn;
  public MyButton defaultButton, upButton, groundButton;

  // Commands
  public RawTankTeleop tankTeleop;
  public RawPivotTeleop pivotTeleop;
  public RawElevatorTeleop elevatorTeleop;
  public ClawTeleop clawTeleop;

  // Preset Commands
  public DefaultPreset defaultPreset;
  public GroundPreset groundPreset;
  public UpPreset upPreset;

  // Auto Commands
  DoNothing doNothingAuto;

  MidCubeShot midCubeShot;
  HighCubeShot highCubeShot;
  ShootAndLeaveAuto shootAndLeaveAuto;
  ChargePanelAuto chargePanelAuto;
  ShootAndChargeAuto shootAndChargeAuto;
  LeaveAndChargeAuto leaveAndCharge;
  LiterallyEverything everythingAuto;

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

    elevatorMotor = new MySparkMax(RobotMap.ELEVATOR_CAN_ID, true, RobotMap.ELEVATOR_REVERSED);
    elevatorMotor.brake();

    clawRoller1 = new VictorSP(RobotMap.CLAW_ROLLER1);
    clawRoller1.setInverted(RobotMap.CLAW_REVERSED);
    clawRoller2 = new VictorSP(RobotMap.CLAW_ROLLER2);
    clawRoller2.setInverted(!RobotMap.CLAW_REVERSED);

    clawWrist = new VictorSP(RobotMap.CLAW_WRIST);
    clawWrist.setInverted(RobotMap.WRIST_REVERSED);
//
    // Pneumatics
    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLAW_IN, RobotMap.CLAW_OUT);

    // Sensors
    gyro = new AHRS(SPI.Port.kMXP);
    pivotEncoder = new Encoder(RobotMap.PIVOT_ENCODER_IN, RobotMap.PIVOT_ENCODER_OUT);
    wristEncoder = new Encoder(RobotMap.WRIST_ENCODER_IN, RobotMap.WRIST_ENCODER_OUT);

    // Subsystems
    drive = new TankDrive(leftFront, rightFront, gyro);
    pivot = new Pivot(leftPivot, rightPivot, pivotEncoder);
    elevator = new Elevator(elevatorMotor);
    claw = new Claw(clawSolenoid, clawRoller1, clawRoller2, clawWrist, wristEncoder);

    // OI + Buttons
    oi = new OI();
    clawOpenBtn = new MyButton(oi.getOperatorXbox(), OI.XBOX_A);
    clawRollerBtn = new MyButton(oi.getOperatorXbox(), OI.XBOX_LB);
    clawRollerOppBtn = new MyButton(oi.getOperatorXbox(), OI.XBOX_RB);

    defaultButton = new MyButton(oi.getOperatorXbox(), OI.XBOX_X);
    upButton = new MyButton(oi.getOperatorXbox(), OI.XBOX_Y);
    groundButton = new MyButton(oi.getOperatorXbox(), OI.XBOX_B);

    sensitivityShifter = new MyButton(oi.getDriverXbox(), OI.XBOX_A);
    purpleBtn = new MyButton(oi.getDriverXbox(), OI.XBOX_X);
    yellowBtn = new MyButton(oi.getDriverXbox(), OI.XBOX_B);

    // shootMidBtn = new MyButton(oi.getOperatorXbox(), OI.XBOX_X);
    // shootHighBtn = new MyButton(oi.getOperatorXbox(), OI.XBOX_Y);

    // Commands
    tankTeleop = new RawTankTeleop(
            drive,
            oi::getDriverXboxLeftTrigger, oi::getDriverXboxRightTrigger, oi::getDriverXboxLeftX,
            sensitivityShifter::isPressed, purpleBtn::isPressed, yellowBtn::isPressed
    );
    pivotTeleop = new RawPivotTeleop(pivot, oi::getOperatorXboxLeftY);
    elevatorTeleop = new RawElevatorTeleop(elevator, oi::getOperatorXboxRightTrigger, oi::getOperatorXboxLeftTrigger);
    clawTeleop = new ClawTeleop(
            claw,
            clawOpenBtn::isPressed, clawRollerBtn::getRaw, clawRollerOppBtn::getRaw, oi::getOperatorXboxRightY
    );

    // presets here
    defaultPreset = new DefaultPreset(pivot, elevator, claw);
    groundPreset = new GroundPreset(pivot, elevator, claw);
    upPreset = new UpPreset(pivot, elevator, claw);
    
    // Auto Commands
    doNothingAuto = new DoNothing();
    midCubeShot = new MidCubeShot(claw);
    highCubeShot = new HighCubeShot(claw);
    shootAndLeaveAuto = new ShootAndLeaveAuto(drive, claw);
    chargePanelAuto = new ChargePanelAuto(drive, false);
    shootAndChargeAuto = new ShootAndChargeAuto(drive, claw);
    leaveAndCharge = new LeaveAndChargeAuto(drive);
    everythingAuto = new LiterallyEverything(drive, claw);

    // Sendable Chooser:
    chooser = new SendableChooser<>();

    SmartDashboard.putData(chooser);

    chooser.addOption("Do Nothing Auto", doNothingAuto);
    chooser.addOption("Shoot Test Auto", highCubeShot);
    chooser.addOption("Shoot And Leave Auto", shootAndLeaveAuto);
    chooser.addOption("JUST (Old) Charge Panel Auto", chargePanelAuto);
    chooser.addOption("Shoot And Charge Auto", shootAndChargeAuto);
    chooser.addOption("Leave Community + Charge Auto", leaveAndCharge);
    chooser.addOption("Do Everything", everythingAuto);

    configureButtonBindings();
  }

  public void configureButtonBindings() {
    // shootMidBtn.toggleOnTrue(midCubeShot);
    // shootHighBtn.toggleOnTrue(highCubeShot);
    defaultButton.toggleOnTrue(defaultPreset);
    upButton.toggleOnTrue(upPreset);
    groundButton.toggleOnTrue(groundPreset);
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}