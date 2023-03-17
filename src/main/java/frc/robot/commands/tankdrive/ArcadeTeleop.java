package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArcadeTeleop extends CommandBase {

    private boolean filterEnabled;

    private DoubleSupplier leftInput, rightInput, steeringInput;
    private BooleanSupplier purpleInput, yellowInput;

    private boolean purpleToggle, yellowToggle;

    private final TankDrive driveTrain;

    private final double DEADZONE = 0.12;
    private final double speedSensitivity = 0.4;
    private final double TURNING_GAIN = 1; // TODO mess around with this

    public ArcadeTeleop(
            TankDrive driveTrain,
            DoubleSupplier leftInput,
            DoubleSupplier rightInput,
            DoubleSupplier steeringInput,
            BooleanSupplier purpleInput,
            BooleanSupplier yellowInput
    ) {

        this.driveTrain = driveTrain;
        this.leftInput = leftInput;
        this.rightInput = rightInput;
        this.steeringInput = steeringInput;
        this.purpleInput = purpleInput;
        this.yellowInput = yellowInput;

        addRequirements(driveTrain);
    }

    private double skim(double v) {
        if (v > speedSensitivity) {
            return -((v - speedSensitivity) * TURNING_GAIN);
        } else if (v < speedSensitivity) {
            return -((v + speedSensitivity) * TURNING_GAIN);
        }
        return 0;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double speedPower = 2;
        double steeringPower = 1; // TODO change this depending on driver interest

        double steering = Math.pow(steeringInput.getAsDouble(), steeringPower) * speedSensitivity;
        if (steering > -DEADZONE && steering < DEADZONE) {
            steering = 0;
        }

        double throttle = rightInput.getAsDouble() - leftInput.getAsDouble();
        throttle *= 0.4;

        double rRawPower = throttle - steering;
        double lRawPower = throttle + steering;

        driveTrain.set(lRawPower + skim(lRawPower), rRawPower + skim(rRawPower));

        SmartDashboard.putNumber("steering", steering);
        SmartDashboard.putNumber("throttle", throttle);
        SmartDashboard.putBoolean("purple toggle", purpleToggle);
        SmartDashboard.putBoolean("yellow toggle", yellowToggle);

        if (purpleInput.getAsBoolean()) {
            purpleToggle = !purpleToggle;
        }

        if (yellowInput.getAsBoolean()) {
            yellowToggle = !yellowToggle;
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