package frc.robot.commands.autonomous.tools;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

public class BFDriveStraightAuto extends CommandBase {

    private final TankDrive driveTrain;

    private Timer timer;

    private final double seconds, power;

    public BFDriveStraightAuto(TankDrive driveTrain, double seconds, double power) {
        super();
        this.driveTrain = driveTrain;
        this.seconds = seconds;
        this.power = power;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
        driveTrain.resetEncoders();
        driveTrain.set(power);
    }

    @Override
    public void execute() {
        if (timer.advanceIfElapsed(seconds)) {
            driveTrain.set(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.set(0);
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        return (timer.advanceIfElapsed(seconds));
    }
}