package frc.robot.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

public class MyVictorSPX extends WPI_VictorSPX {

    static class Filter {
        private double sensitivity;
        private double value = 0;

        public Filter(double sensitivity) {
            this.sensitivity = sensitivity;
        }

        public void update(double newValue) {
            value = sensitivity * newValue + (1 - sensitivity) * value;
        }

        public double getValue() {
            return value;
        }

        public void setValue(double value) {
            this.value = value;
        }
    }

    private static final double DEFAULT_PERCENT_FILTER = 1.0;
    private final Filter percentOutputFilter;

    private final Encoder tbEncoder; // through bore encoder

    private static final int TIMEOUT_MS = 30;

    private final MyVictorSPX brownoutFollower = null;
    private boolean brownout = false;

    /**
     * Constructor
     *
     *
     */
    public MyVictorSPX(int channel, boolean reversed, int digIn, int digOut) {
        super(channel);
        super.setInverted(reversed);
        percentOutputFilter = new Filter(DEFAULT_PERCENT_FILTER);
        tbEncoder = new Encoder(digIn, digOut, reversed, CounterBase.EncodingType.k4X); // TODO change k4X if necessary
        resetEncoder();
    }
    public void set(double speed) {
        if (speed > 1) speed = 1;
        if (speed < -1) speed = -1;
        percentOutputFilter.update(speed);
        super.set(ControlMode.PercentOutput, percentOutputFilter.getValue());
    }
    public double getPercentSpeed() {
        return super.getMotorOutputPercent();
    }

    public boolean isStopped() {
        return this.tbEncoder.getStopped();
    }

    public void stop() {
        set(0);
    }

    public void brake() {
        this.set(0);
        super.setNeutralMode(NeutralMode.Brake);
    }
    public void coast() {
        this.set(0);
        super.setNeutralMode(NeutralMode.Coast);
    }
    public int getChannel() {
        return super.getDeviceID();
    }

    public boolean isReversed() {
        return super.getInverted();
    }

    public void setReversed(boolean reversed) {
        super.setInverted(reversed);
    }

    public void setDistancePerPulse(double newDPP) {
        this.tbEncoder.setDistancePerPulse(newDPP);
    }

    public void resetEncoder() {
        this.tbEncoder.reset();
    }

    public double getDistance() {
        return this.tbEncoder.getDistance();
    }

    public double getDirection() {
        return (this.tbEncoder.getDirection()) ? 1 : -1;
    }

    public double getError() {
        return super.getClosedLoopError(0);
    }

    public void enableBrownoutProtection() {
        if (brownoutFollower != null) {
            brownoutFollower.coast();
        }
        brownout = true;
    }

    public void disableBrownoutProtection() {
        if (brownoutFollower != null && brownout) {
            brownoutFollower.setNeutralMode(NeutralMode.Brake);
            brownoutFollower.set(ControlMode.Follower, getChannel());
        }
        brownout = false;
    }





}