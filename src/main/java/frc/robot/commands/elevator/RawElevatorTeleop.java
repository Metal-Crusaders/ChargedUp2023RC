package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;

public class RawElevatorTeleop extends CommandBase {

    public static final double FULL_POWER = 0.5;
    public static final double DEADBAND = 0.15;

    private Elevator elevator;
    private DoubleSupplier forwardInput, backwardInput;

    public RawElevatorTeleop(Elevator elevator, DoubleSupplier forwardInput, DoubleSupplier backwardInput) {

        this.elevator = elevator;
        this.forwardInput = forwardInput;
        this.backwardInput = backwardInput;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stop();
    }

    @Override
    public void execute() {

        double elevatorPower = 1;

        double rawInp = forwardInput.getAsDouble() - backwardInput.getAsDouble();

        double speed = Math.pow(rawInp, elevatorPower) * FULL_POWER;
        if (speed < DEADBAND && speed > -DEADBAND) {
            speed = 0;
        }

        if (
            (elevator.upperLimitTriggered() && speed >= 0) ||
            (elevator.lowerLimitTriggered() && speed <= 0)
        ) {
            speed = 0;
        }

        SmartDashboard.putNumber("Elevator Speed", speed);
        SmartDashboard.putBoolean("Lower Limit Triggered", elevator.lowerLimitTriggered());
        SmartDashboard.putBoolean("Upper Limit Triggered", elevator.upperLimitTriggered());

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