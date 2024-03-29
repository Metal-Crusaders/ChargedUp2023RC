// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.TankDrive;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer timer;

  boolean pathScheduled = false;

  private void resetStuff() {
    m_robotContainer.drive.resetEncoders();
    m_robotContainer.drive.getGyro().reset();
    m_robotContainer.drive.brake();

    m_robotContainer.pivot.resetEncoder();

    m_robotContainer.elevator.stop();
    m_robotContainer.elevator.resetEncoder();

  //  m_robotContainer.claw.set(false);
    m_robotContainer.claw.setRollers(false, false);
    m_robotContainer.claw.setWrist(0);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard
    m_robotContainer = new RobotContainer();

    // resetStuff();

    timer = new Timer();

    CommandScheduler.getInstance().setDefaultCommand(m_robotContainer.drive, m_robotContainer.tankTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_robotContainer.pivot, m_robotContainer.pivotTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_robotContainer.elevator, m_robotContainer.elevatorTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_robotContainer.claw, m_robotContainer.clawTeleop);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.drive.coast();

    m_robotContainer.pivot.stop();
    m_robotContainer.elevator.stop();

//    m_robotContainer.claw.set(false);
    m_robotContainer.claw.setRollers(false, false);
    m_robotContainer.claw.setWrist(0);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    resetStuff();

    timer.reset();
    timer.start();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      pathScheduled = true;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    CommandScheduler.getInstance().run();

    if (timer.hasElapsed(0.1) && !pathScheduled){
      m_autonomousCommand.schedule();
      pathScheduled = true;
    }

    if (m_autonomousCommand.isFinished()) {
      m_autonomousCommand.end(true);
      disabledInit();
    }

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    m_robotContainer.drive.brake();

    m_robotContainer.pivot.stop();
    m_robotContainer.elevator.stop();

//    m_robotContainer.claw.set(false);
    m_robotContainer.claw.setRollers(false, false);
    m_robotContainer.claw.setWrist(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Speed of LeftFront: ", m_robotContainer.leftFront.getSpeed(TankDrive.NUM_TICKS_PER_ROTATION));
    SmartDashboard.putNumber("Speed of LeftRear: ", m_robotContainer.leftRear.getSpeed(TankDrive.NUM_TICKS_PER_ROTATION));
    SmartDashboard.putNumber("Speed of RightFront: ", m_robotContainer.rightFront.getSpeed(TankDrive.NUM_TICKS_PER_ROTATION));
    SmartDashboard.putNumber("Speed of RightRear: ", m_robotContainer.rightRear.getSpeed(TankDrive.NUM_TICKS_PER_ROTATION));
    SmartDashboard.putBoolean("Is Right Front (2) Inverted: ", m_robotContainer.rightFront.getInverted());
    SmartDashboard.putBoolean("Is Right Rear (3) Inverted: ", m_robotContainer.rightRear.getInverted());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
