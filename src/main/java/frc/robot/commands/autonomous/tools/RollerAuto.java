package frc.robot.commands.autonomous.tools;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class RollerAuto extends CommandBase{
    
    private final Claw claw;

    private Timer timer;

    private final double seconds;
    private final boolean in;

    public RollerAuto(Claw claw, double seconds, boolean in) {
        super();
        this.claw = claw;
        this.seconds = seconds;
        this.in = in;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
        claw.setRollers(in, !in);
    }

    @Override
    public void execute() {
        if (timer.get() > (seconds)) {
            claw.setRollers(false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.setRollers(false, false);
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        return (timer.get() > seconds);
    }
}
