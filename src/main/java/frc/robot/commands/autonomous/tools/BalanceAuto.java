package frc.robot.commands.autonomous.tools;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

public class BalanceAuto extends CommandBase {

    private final TankDrive driveTrain;

    private final double deadband = 1;

    private final double FULL_POWER = 0.075;

    private final double kP = (1.0 / 12);

    private boolean isBackwards;

    public BalanceAuto(TankDrive driveTrain, boolean isBackwards) {
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

        double speed = (driveTrain.getTilt() * kP) * FULL_POWER;

        if (!isBackwards) {
            speed *= -1;
        }

        SmartDashboard.putNumber("Balance Speed", speed);

        driveTrain.set(speed);

    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.set(0);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
//        return (driveTrain.getTilt() < deadband) && (driveTrain.getTilt() > -deadband);
        return false;
    }
}