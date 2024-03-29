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

    private final double DEADZONE = 0.12;
    private final double LOW_SPEED = 0.4;
    private final double HIGH_SPEED = 0.85;

    double speedSensitivity = LOW_SPEED;
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

        double steering = Math.pow(steeringInput.getAsDouble(), steeringPower);
        if (steering > -DEADZONE && steering < DEADZONE) {
            steering = 0;
        }

        steering *= 0.7;
        
        double throttle = rightInput.getAsDouble() - leftInput.getAsDouble();

        throttle *= speedSensitivity;

        double rRawPower = (throttle + steering);
        double lRawPower = (throttle - steering);

        double rightSign = (rRawPower == 0) ? 0 : (rRawPower) / Math.abs(rRawPower);
        double leftSign = (lRawPower == 0) ? 0 : (lRawPower) / Math.abs(lRawPower);

        double rpower = rightSign * Math.pow(rRawPower, speedPower);
        double lpower = leftSign * Math.pow(lRawPower, speedPower);

        driveTrain.set(lpower, rpower);

        SmartDashboard.putNumber("steering", steering);
        SmartDashboard.putNumber("throttle", throttle);
        SmartDashboard.putNumber("speedSense", speedSensitivity);
        SmartDashboard.putNumber("lpower", lpower);
        SmartDashboard.putNumber("rpower", rpower);
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
            if (speedSensitivity == HIGH_SPEED) {
                speedSensitivity = LOW_SPEED;
            } else {
                speedSensitivity = HIGH_SPEED;
            }
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