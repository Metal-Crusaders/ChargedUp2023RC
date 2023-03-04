package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.MyButton;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.BooleanSupplier;

public class ClawTeleop extends CommandBase {

    Claw claw;
    BooleanSupplier openBtn, rollerBtn;

    private boolean toggleClaw, toggleRollers;

    public ClawTeleop(Claw claw, BooleanSupplier openBtn, BooleanSupplier rollerBtn) {

        this.claw = claw;
        this.openBtn = openBtn;
        this.rollerBtn = rollerBtn;

        toggleClaw = false;
        toggleRollers = false;

        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        claw.set(false);
        claw.setRollers(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (openBtn.getAsBoolean()) {
            toggleClaw = !toggleClaw;
        }

        if (rollerBtn.getAsBoolean()) {
            toggleRollers = !toggleRollers;
        }

        claw.set(toggleClaw);

        claw.setRollers(toggleRollers);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        claw.set(false);
        claw.setRollers(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
