package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static java.lang.Math.abs;

public class ClawTeleop extends CommandBase {

    Claw claw;
    BooleanSupplier rollerBtn, oppRollerBtn;
    DoubleSupplier wristInput;

    double DEADBAND = 0.05;
    double WRIST_FULL_POWER = 0.5;

    private boolean toggleClaw;

    public ClawTeleop(Claw claw, BooleanSupplier rollerBtn, BooleanSupplier oppRollerBtn, DoubleSupplier wristInput) {

        this.claw = claw;
        this.rollerBtn = rollerBtn;
        this.oppRollerBtn = oppRollerBtn;
        this.wristInput = wristInput;

        toggleClaw = false;

        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        claw.setRollers(false, false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        claw.setRollers(rollerBtn.getAsBoolean(), oppRollerBtn.getAsBoolean());

        double wristSpeed = wristInput.getAsDouble();

        if (abs(wristSpeed) < DEADBAND) {
            wristSpeed = -0.1;
        }

        wristSpeed *= WRIST_FULL_POWER;

        SmartDashboard.putBoolean("Claw open?", toggleClaw);
        SmartDashboard.putBoolean("Rollers on?", rollerBtn.getAsBoolean());
        SmartDashboard.putBoolean("Anti_speed rollers?", oppRollerBtn.getAsBoolean());
        SmartDashboard.putNumber("Wrist Speed", wristSpeed);
        SmartDashboard.putNumber("Wrist Encoders: ", claw.getWristTicks());

        claw.setWrist(wristSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        claw.setRollers(false, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
