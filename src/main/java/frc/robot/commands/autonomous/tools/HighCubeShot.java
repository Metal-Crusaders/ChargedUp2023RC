package frc.robot.commands.autonomous.tools;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class HighCubeShot extends CommandBase {

    private final Claw claw;
    private Timer timer;
    private final double BENCHMARK1 = 0.1, BENCHMARK2 = 0.5;

    public HighCubeShot(Claw claw) {
        this.claw = claw;
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
        claw.rawRollerControl(-0.15);
    }

    @Override
    public void execute() {
        if (timer.advanceIfElapsed(BENCHMARK1)) {
            claw.rawRollerControl(0.9);
        }
        if (timer.advanceIfElapsed(BENCHMARK2)) {
            claw.rawRollerControl(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.rawRollerControl(0);
    }

    @Override
    public boolean isFinished() {
        return (timer.advanceIfElapsed(BENCHMARK2));
    }
}