package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;

public class RawElevatorTeleop extends CommandBase {

    public static final int LOWER_BOUND = -5, UPPER_BOUND = 425;
    // TODO double check to make sure Elevator bounds are working
    public static final double FULL_POWER = 0.2;
    public static final double DEADBAND = 0.05;

    private Elevator elevator;
    private DoubleSupplier elevatorInput;

    public RawElevatorTeleop(Elevator elevator, DoubleSupplier elevatorInput) {

        this.elevator = elevator;
        this.elevatorInput = elevatorInput;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stop();
    }

    @Override
    public void execute() {
        double speed = elevatorInput.getAsDouble() * FULL_POWER;
        SmartDashboard.putNumber("Elevator Speed", speed);
//        SmartDashboard.putNumber("Elevator Encoder Ticks", elevator.getDistance());

        if (speed < DEADBAND && speed > -DEADBAND) {
            speed = 0;
        }

        if (
            (elevator.upperLimitTriggered() && speed >= 0) ||
            (elevator.lowerLimitTriggered() && speed <= 0)
        ) {
            speed = 0;
        }

        elevator.set(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }


}