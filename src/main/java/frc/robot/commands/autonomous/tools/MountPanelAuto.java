package frc.robot.commands.autonomous.tools;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

public class MountPanelAuto extends CommandBase {

    private final TankDrive driveTrain;

    private final double deadband = 15;

    private final double FULL_POWER = 0.3;

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

        double speed = FULL_POWER;

        if (isBackwards) {
            speed *= -1;
        }

        driveTrain.set(speed);
        SmartDashboard.putNumber("Gyro Angle Mounting", driveTrain.getTilt());
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.set(0);
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        return ((driveTrain.getTilt() > deadband) || (driveTrain.getTilt() < -deadband));
    }
}