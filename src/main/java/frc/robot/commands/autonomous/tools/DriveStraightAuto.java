package frc.robot.commands.autonomous.tools;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

import static java.lang.Math.max;
import static java.lang.Math.min;

public class DriveStraightAuto extends CommandBase {

    private final TankDrive driveTrain;

    private final double FULL_POWER = 0.25;
    private final double PRECISION = 60;
    // TODO get better precision value here (based on good math estimate)
    double target;
    double error = 0;
    double current = 0;

    PIDController pidController;

    double kP = 0.0009; // kP when FULL_POWER is 0.5 is 0.00045
    double kI = 0;
    double kD = 0;

    public DriveStraightAuto(TankDrive driveTrain, double target) {
        super();
        this.driveTrain = driveTrain;
        this.target = target;
        this.target /= TankDrive.NUM_INCHES_PER_ROTATION;
        this.target *= TankDrive.NUM_TICKS_PER_ROTATION;
        this.pidController = new PIDController(kP, kI, kD);
        pidController.setSetpoint(this.target);
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetEncoders();
        pidController.reset();
    }

    @Override
    public void execute() {
//        SmartDashboard.putData("PID Controller: ", pidController);
        current = driveTrain.getAverageEncoderDistance();
        SmartDashboard.putNumber("avg dt", current);
        error = target - current;
        SmartDashboard.putNumber("Error drive straight", error);
        double power = min(1, max(-1, pidController.calculate(current))) * FULL_POWER;
        SmartDashboard.putNumber("speed auto", power);
        driveTrain.set(power);
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
        return (Math.abs(error) < PRECISION);
//        return Math.abs(driveTrain.getAverageEncoderDistance()) == target;
    }
}