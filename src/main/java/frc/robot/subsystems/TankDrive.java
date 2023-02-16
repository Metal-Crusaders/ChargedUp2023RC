package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MySparkMax;
import com.kauailabs.navx.frc.AHRS;

public class TankDrive extends SubsystemBase {

    public final static double NUM_TICKS_PER_ROTATION = 3550;
    public final static double NUM_INCHES_PER_ROTATION = 6 * Math.PI;
    private final int DRIVETRAIN_INCHES_PER_PULSE = 3;

    private final MySparkMax left, right;
    private final AHRS gyro;

    // PID Stuff
    private DifferentialDriveOdometry odometry;
    private PIDController drivePID;

    public TankDrive(MySparkMax left, MySparkMax right, AHRS gyro) {
        this.left = left;
        this.right = right;
        this.gyro = gyro;

        resetEncoders();
        gyro.reset();

        // PID Stuff
        this.drivePID = new PIDController(0, 0, 0);
        this.odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);
    }

    // Motor setters
    public void set(double leftSpeed, double rightSpeed) {
        left.set(leftSpeed);
        right.set(rightSpeed);
    }

    public void set(double speed) {
        this.set(speed, speed);
    }

    public void turnInPlace(boolean ifRight, double speed) {
        if (ifRight) {
            set(speed, -speed);
        } else {
            set(-speed, speed);
        }
    }

    public void stop() {
        this.set(0);
    }

    public void brake() {
        left.brake();
        right.brake();
    }

    public void coast() {
        left.coast();
        right.coast();
    }

    // accessors
    public MySparkMax getLeft() {
        return left;
    }

    public MySparkMax getRight() {
        return right;
    }

    // Positioning Sensor Methods:
    public AHRS getGyro() {
        return gyro;
    }

    public double getYaw() {
        return gyro.getRotation2d().getDegrees();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetEncoders() {
        left.resetEncoder();
        right.resetEncoder();
    }

    public double getLeftDistance() {
        return left.getDistance();
    }

    public double getRightDistance() {
        return right.getDistance();
    }

    public double getAverageEncoderDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    // PID Stuff
    @Override
    public void periodic(){
        //updates odometry periodically
        odometry.update(
                gyro.getRotation2d(),
                getLeftDistance() * DRIVETRAIN_INCHES_PER_PULSE * 0.0254,
                -getRightDistance() * DRIVETRAIN_INCHES_PER_PULSE * 0.0254
        );

		SmartDashboard.putNumber("NavX Gyro Angle (deg)", getYaw());
		SmartDashboard.putNumber("Odometry X (m)", odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("Odometry Y (m)", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Average Encoder Distance", getAverageEncoderDistance());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(
                getLeftDistance() * 10 * DRIVETRAIN_INCHES_PER_PULSE * 0.0254,
                -getRightDistance() * 10 * DRIVETRAIN_INCHES_PER_PULSE * 0.0254
        );
    }

}
