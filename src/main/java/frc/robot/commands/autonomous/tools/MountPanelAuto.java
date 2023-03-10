package frc.robot.commands.autonomous.tools;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

public class MountPanelAuto extends CommandBase {

    private final TankDrive driveTrain;

    private final double deadband = 5;

    private final double FULL_POWER = 0.2;

    private boolean isBackwards;

    public MountPanelAuto(TankDrive driveTrain, boolean isBackwards) {
        super();
        this.driveTrain = driveTrain;
        this.isBackwards = isBackwards;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetEncoders();
        driveTrain.resetGyro();
    }

    @Override
    public void execute() {
        driveTrain.set(FULL_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.set(0);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        if (isBackwards) {
            return (driveTrain.getTilt() > deadband);
        } else {
            return (driveTrain.getTilt() < -deadband);
        }
    }
}