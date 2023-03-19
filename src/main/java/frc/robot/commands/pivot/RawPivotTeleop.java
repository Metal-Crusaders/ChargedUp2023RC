package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

import java.util.function.DoubleSupplier;

public class RawPivotTeleop extends CommandBase {

    public static final double FULL_POWER = 0.35;
    public static final double DEADZONE = 0.085;

    private Pivot pivot;
    private DoubleSupplier pivotInput;

    public RawPivotTeleop(Pivot pivot, DoubleSupplier pivotInput) {

        this.pivot = pivot;
        this.pivotInput = pivotInput;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
//        pivot.resetEncoder();
    }

    @Override
    public void execute() {
        double speed = pivotInput.getAsDouble() * FULL_POWER;
        SmartDashboard.putNumber("Pivot Speed", speed);
        SmartDashboard.putNumber("Pivot Encoder Ticks", pivot.getEncoderTicks());

        if (speed < DEADZONE && speed > -DEADZONE) {
            speed = 0;
        }

        pivot.set(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.pivot.stop();
    }

}