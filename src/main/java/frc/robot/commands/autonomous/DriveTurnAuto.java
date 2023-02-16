package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

import static java.lang.Math.max;
import static java.lang.Math.min;

public class DriveTurnAuto extends CommandBase {

    private final TankDrive driveTrain;

    private final double FULL_POWER = 0.25;
    private final double PRECISION = 5;

    double target;
    double current;
    PIDController pidController;
    double kP = 0.02;
    double kI = 0;
    double kD = 0;

    public DriveTurnAuto(TankDrive driveTrain, double target) {
        super();
        this.driveTrain = driveTrain;
        driveTrain.getGyro().reset();

        this.target = -1 * target;

        this.pidController = new PIDController(kP, kI, kD);
        pidController.setSetpoint(this.target);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetEncoders();
        driveTrain.resetGyro();
        pidController.reset();
    }

    @Override
    public void execute() {
//        SmartDashboard.putData("PID Controller: ", pidController);
        current = driveTrain.getYaw();
        SmartDashboard.putNumber("target auto", target);
        SmartDashboard.putNumber("current auto", driveTrain.getYaw());
        double power = (Math.abs(this.target) / this.target) * min(1, max(-1, pidController.calculate(current))) * FULL_POWER;
        driveTrain.turnInPlace(target > 0, power);
        SmartDashboard.putBoolean("finished first swivel", isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.set(0);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(driveTrain.getGyro().getYaw()) < Math.abs(target) + PRECISION) &&
                (Math.abs(driveTrain.getGyro().getYaw()) > Math.abs(target) - PRECISION);
    }
}
