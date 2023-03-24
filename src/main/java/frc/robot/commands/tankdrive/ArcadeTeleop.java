package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArcadeTeleop extends CommandBase {

    private boolean filterEnabled;

    private final DoubleSupplier leftInput, rightInput, steeringInput;
    private final BooleanSupplier sensiInput, purpleInput, yellowInput;

    private boolean sensiToggle, purpleToggle, yellowToggle;

    private final TankDrive driveTrain;

    protected final double DEADZONE = 0.12;
    private double speedSensitivity = 0.85;
    protected final double TURNING_GAIN = 1; // TODO mess around with this

    public ArcadeTeleop(
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

    private double skim(double v) {
        if (v > speedSensitivity) {
            return -((v - speedSensitivity) * TURNING_GAIN);
        } else if (v < -speedSensitivity) {
            return -((v + speedSensitivity) * TURNING_GAIN);
        }
        return 0;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

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

        double speedPower = 2;
        double steeringPower = 1; // TODO change this depending on driver interest

        double steering = Math.pow(steeringInput.getAsDouble(), steeringPower) * speedSensitivity;
        if (steering > -DEADZONE && steering < DEADZONE) {
            steering = 0;
        }

        double throttle = Math.pow((rightInput.getAsDouble() - leftInput.getAsDouble()), speedPower) * speedSensitivity;

        double rRawPower = throttle - steering;
        double lRawPower = throttle + steering;

        driveTrain.set(lRawPower + skim(lRawPower), rRawPower + skim(rRawPower));

        SmartDashboard.putNumber("steering", steering);
        SmartDashboard.putNumber("throttle", throttle);
        SmartDashboard.putBoolean("purple toggle", purpleToggle);
        SmartDashboard.putBoolean("yellow toggle", yellowToggle);

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