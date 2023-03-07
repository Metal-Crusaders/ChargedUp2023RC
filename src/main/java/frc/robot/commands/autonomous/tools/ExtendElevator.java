package frc.robot.commands.autonomous.tools;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class RestElevator extends CommandBase {

    private Elevator elevator;

    private final double FULL_POWER = 0.25;

    public RestElevator(Elevator elevator) {
        super();
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stop();
    }

    @Override
    public void execute() {
        elevator.set(FULL_POWER);
    }

    @Override
    public void end(boolean interrupted) {
      elevator.stop();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.lowerLimitTriggered();
    }


}
