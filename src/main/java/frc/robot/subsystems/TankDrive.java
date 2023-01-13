package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MySparkMax;

public class TankDrive extends SubsystemBase {

    private final MySparkMax left, right;

    public TankDrive(MySparkMax left, MySparkMax right) {
        this.left = left;
        this.right = right;
    }

    public void set(double leftSpeed, double rightSpeed) {
        left.set(leftSpeed);
        right.set(rightSpeed);
    }

    public void set(double speed) {
        this.set(speed, speed);
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

    public MySparkMax getLeft() {
        return left;
    }

    public MySparkMax getRight() {
        return right;
    }
}
