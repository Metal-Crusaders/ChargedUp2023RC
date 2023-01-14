package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

import java.util.function.DoubleSupplier;

public class RawTankTeleop extends CommandBase {


    private boolean filterEnabled;

    private DoubleSupplier leftInput, rightInput, steeringInput;

    private final TankDrive driveTrain;

    private final double DEADBAND = 0.12;

    // TODO INVERSE DRIVE (1/X) implementation
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

        double steering = steeringInput.getAsDouble();
        if (steering > -DEADBAND && steering < DEADBAND) {
            steering = 0;
        }

        double throttle = rightInput.getAsDouble() - leftInput.getAsDouble();
        double rpower = throttle * (1 + steering) * speedSensitivity;
        double lpower = throttle * (1 - steering) * speedSensitivity;

        driveTrain.set(lpower, rpower);
        SmartDashboard.putNumber("steering", steering);
        SmartDashboard.putNumber("throttle", rightInput.getAsDouble() - leftInput.getAsDouble());
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