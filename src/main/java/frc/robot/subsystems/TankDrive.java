package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MySparkMax;
import com.kauailabs.navx.frc.AHRS;



public class TankDrive extends SubsystemBase {

    public final static double NUM_TICKS_PER_ROTATION = 3550;
    public final static double NUM_INCHES_PER_ROTATION = 6 * Math.PI;
    public final double DRIVETRAIN_INCHES_PER_PULSE = (NUM_INCHES_PER_ROTATION / NUM_TICKS_PER_ROTATION);

    public final static int NUM_LEDS = 30;

    private final MySparkMax left, right;
    private final AHRS gyro;
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuf;

    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;

    // PID Stuff
    private DifferentialDriveOdometry odometry;
    private PIDController drivePID;

    public TankDrive(MySparkMax left, MySparkMax right, AHRS gyro, AddressableLED leds) {
        this.left = left;
        this.right = right;
        this.gyro = gyro;

        this.leds = leds;
        this.ledBuf = new AddressableLEDBuffer(NUM_LEDS);
        this.leds.setData(ledBuf);
        this.leds.start();

        resetEncoders();
        this.gyro.reset();

        // PID Stuff
        this.drivePID = new PIDController(kP, kI, kD);
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

    public double getTilt() {
        return gyro.getPitch();
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

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    // LED methods
    public void setColor(int r, int g, int b) {
        for (int i = 0; i < ledBuf.getLength(); i++) {
            ledBuf.setRGB(i, 255, 0, 0);
        }
        leds.setData(ledBuf);
    }

    public void setPurple() {
        setColor(125, 41, 242);
    }

    public void setYellow() {
        setColor(235, 215, 67);
    }

    public void lightsOff() {
        setColor(0, 0, 0);
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
