package frc.robot.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

public class MySparkMax extends CANSparkMax  {

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId  The device ID.
     * @param brushless Sets the motor type to brushed / brushless. (boolean)
     */

    public MySparkMax(int deviceId, boolean brushless) {
        super(deviceId, ((brushless) ? MotorType.kBrushless : MotorType.kBrushed));
    }

    public MySparkMax(int deviceId, boolean brushless, boolean inverted) {
        super(deviceId, ((brushless) ? MotorType.kBrushless : MotorType.kBrushed));
        this.setInverted(inverted);
    }

    public MySparkMax getMotor() {
        return this;
    }

    public void set(double speed) {
        if (speed > 1) {
            speed = 1;
        }
        if (speed < -1) {
            speed = -1;
        }

        super.set(speed);
    }

    public void setInverted(boolean inverted) {
        super.setInverted(inverted);
    }

    public boolean getInverted() {
        return super.getInverted();
    }

    public void stop() {
        this.set(0);
    }

    public void brake() {
        this.stop();
        super.setIdleMode(IdleMode.kBrake);
    }

    public void coast() {
        this.stopMotor();
        super.setIdleMode(IdleMode.kCoast);
    }

    public void follow(MySparkMax parent) {
        super.follow(parent);
    }

    // encoder stuff
    @Override
    public RelativeEncoder getEncoder() {
        return super.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    }
    @Override
    public RelativeEncoder getEncoder(SparkMaxRelativeEncoder.Type encoderType, int countsPerRev) {
        return super.getEncoder(encoderType, countsPerRev);
    }

    public void setEncoderSettings(double positionConstant, double velocityConstant) {
        this.getEncoder().setPositionConversionFactor(positionConstant);
        this.getEncoder().setVelocityConversionFactor(velocityConstant);
    }

    public void resetEncoder() {
        this.getEncoder().setPosition(0);
    }

    public double getDistance() {
        return this.getEncoder().getPosition();
    }

    public double getSpeed() {
        return this.getEncoder().getVelocity();
    }

}
