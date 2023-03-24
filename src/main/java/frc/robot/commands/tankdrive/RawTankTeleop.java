package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RawTankTeleop extends CommandBase {

    private boolean filterEnabled;

    private DoubleSupplier leftInput, rightInput, steeringInput;
    private BooleanSupplier sensiInput, purpleInput, yellowInput;

    private boolean sensiToggle, purpleToggle, yellowToggle;

    private final TankDrive driveTrain;

    private final double DEADZONE = 0.08;

    double speedSensitivity = 0.4;
    double speedPower = 2;
    double steeringPower = 1; // TODO change this depending on driver interest

    public RawTankTeleop(
            TankDrive driveTrain,
            DoubleSupplier leftInput,
            DoubleSupplier rightInput,
            DoubleSupplier steeringInput,
            BooleanSupplier sensiInput,
            BooleanSupplier purpleInput,
            BooleanSupplier yellowInput
    ) {

        this.driveTrain = driveTrain;
        this.leftInput = leftInput;
        this.rightInput = rightInput;
        this.steeringInput = steeringInput;
        this.sensiInput = sensiInput;
        this.purpleInput = purpleInput;
        this.yellowInput = yellowInput;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double steering = Math.pow(steeringInput.getAsDouble(), steeringPower) * speedSensitivity;
        if (steering > -DEADZONE && steering < DEADZONE) {
            steering = 0;
        }

        double throttle = rightInput.getAsDouble() - leftInput.getAsDouble();

        double rRawPower = (throttle - steering) * speedSensitivity;
        double lRawPower = (throttle + steering) * speedSensitivity;

        if (throttle == 0) {
            rRawPower = -steering;
            lRawPower = steering;
        }

        double rightSign = (rRawPower == 0) ? 0 : (rRawPower) / Math.abs(rRawPower);
        double leftSign = (lRawPower == 0) ? 0 : (lRawPower) / Math.abs(lRawPower);

        double rpower = rightSign * Math.pow(rRawPower, speedPower);
        double lpower = leftSign * Math.pow(lRawPower, speedPower);

        driveTrain.set(rpower, lpower);

        SmartDashboard.putNumber("steering", steering);
        SmartDashboard.putNumber("throttle", throttle);
        SmartDashboard.putBoolean("fast mode toggled", sensiToggle);
        SmartDashboard.putBoolean("purple toggle", purpleToggle);
        SmartDashboard.putBoolean("yellow toggle", yellowToggle);

        if (purpleInput.getAsBoolean()) {
            purpleToggle = !purpleToggle;
        }

        if (yellowInput.getAsBoolean()) {
            yellowToggle = !yellowToggle;
        }

        if (sensiInput.getAsBoolean()) {
            sensiToggle = !sensiToggle;
        }

        if (sensiToggle && speedSensitivity == 0.85) {
            speedSensitivity = 0.4;
        } else {
            speedSensitivity = 0.85;
        }

        if (purpleToggle) {
            // need to set to color purple here
        }

        if (yellowToggle) {
            // yellow here
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.driveTrain.stop();
    }


}