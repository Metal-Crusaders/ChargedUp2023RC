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

    private final int DRIVETRAIN_INCHES_PER_PULSE = 0;
    public static final double kPDriveVel = 8.5;

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

		SmartDashboard.putNumber("NavX Gryo Angle (deg)", gyro.getRotation2d().getDegrees());
		SmartDashboard.putNumber("Odometry X (m)", odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("Odometry Y (m)", odometry.getPoseMeters().getY());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(
                getLeftDistance() * 10 * DRIVETRAIN_INCHES_PER_PULSE * 0.0254,
                -getRightDistance() * 10 * DRIVETRAIN_INCHES_PER_PULSE * 0.0254
        );
    }

}