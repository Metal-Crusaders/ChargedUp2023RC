package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static java.lang.Math.abs;

public class ClawTeleop extends CommandBase {

    Claw claw;
    BooleanSupplier openBtn, rollerBtn, oppRollerBtn;
    DoubleSupplier wristInput;

    double DEADBAND = 0.05;
    double WRIST_FULL_POWER = 0.5;

    private boolean toggleClaw;

    public ClawTeleop(Claw claw, BooleanSupplier openBtn, BooleanSupplier rollerBtn, BooleanSupplier oppRollerBtn, DoubleSupplier wristInput) {

        this.claw = claw;
        this.openBtn = openBtn;
        this.rollerBtn = rollerBtn;
        this.oppRollerBtn = oppRollerBtn;
        this.wristInput = wristInput;

        toggleClaw = false;

        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        claw.set(false);
        claw.setRollers(false);
        claw.setRollersOpposite(false);
        claw.resetWristEncoder();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (openBtn.getAsBoolean()) {
            toggleClaw = !toggleClaw;
        }

        claw.set(toggleClaw);

        claw.setRollers(rollerBtn.getAsBoolean());
        if (!rollerBtn.getAsBoolean()) {
            claw.setRollersOpposite(oppRollerBtn.getAsBoolean());
        }

        double wristSpeed = wristInput.getAsDouble();

        if (abs(wristSpeed) < DEADBAND) {
            wristSpeed = 0;
        }

        wristSpeed *= WRIST_FULL_POWER;

        SmartDashboard.putBoolean("Claw open?", toggleClaw);
        SmartDashboard.putBoolean("Rollers on?", rollerBtn.getAsBoolean());
        SmartDashboard.putBoolean("Anti_speed rollers?", oppRollerBtn.getAsBoolean());
        SmartDashboard.putNumber("Wrist Speed", wristSpeed);

        claw.setWrist(wristSpeed);
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
