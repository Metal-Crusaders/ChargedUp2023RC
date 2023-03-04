package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

import java.util.function.DoubleSupplier;

public class RawTankTeleop extends CommandBase {

    private boolean filterEnabled;

    private DoubleSupplier leftInput, rightInput, steeringInput;

    private final TankDrive driveTrain;

    private final double DEADZONE = 0.12;

    public RawTankTeleop(TankDrive driveTrain, DoubleSupplier leftInput, DoubleSupplier rightInput, DoubleSupplier steeringInput) {

        this.driveTrain = driveTrain;
        this.leftInput = leftInput;
        this.rightInput = rightInput;
        this.steeringInput = steeringInput;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speedSensitivity = 1;
        double steeringSensitivity = 0.4;
        double speedPower = 2;
        double steeringPower = 1; // TODO change this depending on driver interest

        double steering = Math.pow(steeringInput.getAsDouble(), steeringPower) * steeringSensitivity;
        if (steering > -DEADZONE && steering < DEADZONE) {
            steering = 0;
        }

        double throttle = rightInput.getAsDouble() - leftInput.getAsDouble();

        double rRawPower = (throttle - (throttle * steering)) * speedSensitivity;
        double lRawPower = (throttle + (throttle * steering)) * speedSensitivity;

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