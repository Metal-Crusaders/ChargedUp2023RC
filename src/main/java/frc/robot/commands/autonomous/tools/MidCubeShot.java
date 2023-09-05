package frc.robot.commands.autonomous.tools;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class MidCubeShot extends CommandBase {

    private final Claw claw;
    private Timer timer;
    private final double BENCHMARK1 = 0.5, BENCHMARK2 = 1.5;

    public MidCubeShot(Claw claw) {
        super();
        this.claw = claw;
        timer = new Timer();

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        claw.set(true);
    }

    @Override
    public void execute() {
        if (timer.get() >= BENCHMARK1) {
            claw.rawRollerControl(1);
        } else {
            claw.rawRollerControl(-0.35);
        }

        SmartDashboard.putBoolean("High Cube Command Ended", timer.get() >= BENCHMARK2);
        SmartDashboard.putNumber("Timer Seconds", timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        claw.set(true);
        claw.rawRollerControl(0);
    }

    @Override
    public boolean isFinished() {
        return (timer.get() >= BENCHMARK2);
    }
}