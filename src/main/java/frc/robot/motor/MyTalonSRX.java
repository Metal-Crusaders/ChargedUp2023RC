package frc.robot.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/*
 * Motor controller wrapper for the talonSRX
 */
public class MyTalonSRX extends WPI_TalonSRX {

    private static final double DEFAULT_PERCENT_FILTER = 1.0;
    private TalonEncoder encoder;
    private static final int TIMEOUT_MS = 30;

    private MyTalonSRX brownoutFollower = null;
    private boolean brownout = false;
    private Filter percentOutputFilter;

    public MyTalonSRX(int channel, boolean reversed) {
        super(channel);
        super.setInverted(reversed);
        percentOutputFilter = new Filter(DEFAULT_PERCENT_FILTER);
    }

    public MyTalonSRX(int channel, boolean reversed, TalonEncoder encoder) {
        super(channel);
        super.setInverted(reversed);

        this.encoder = encoder;
    }

    @Override
    public void set(double speed) {
        if (speed > 1) speed = 1;
        if (speed < -1) speed = -1;
        percentOutputFilter.update(speed);
        super.set(ControlMode.PercentOutput, percentOutputFilter.getValue());
    }

    public void setVelocityRPM(double rpm) {
        double ticksper100ms = rpm * 2048 / (60 * 10);
        this.set(ControlMode.Velocity, ticksper100ms);
    }

    public void setAngleTicks(double ticks) {
        this.set(ControlMode.Position, ticks);
    }

    public void brake() {
        this.set(0);
        super.setNeutralMode(NeutralMode.Brake);
    }

    public void coast() {
        this.set(0);
        super.setNeutralMode(NeutralMode.Coast);
    }

    public boolean hasEncoder() {
        return !(encoder == null);
    }

    public TalonEncoder getEncoder() {
        return encoder;
    }

    public void setEncoder(TalonEncoder encoder) {
        this.encoder = encoder;
    }


    public int getChannel() {
        return super.getDeviceID();
    }


    public boolean isReversed() {
        return super.getInverted();
    }


    public double getPercentSpeed() {
        return super.getMotorOutputPercent();
    }

    public double getCurrent() {
        return super.getStatorCurrent();
    }


    public double getSpeed() {
        if (!hasEncoder())
            return 0;
        return encoder.getSpeed();
    }

    public double getError() {
        return super.getClosedLoopError(0);
    }


    public void stop() {
        set(0);
    }

    public void follow(MyTalonSRX other) {
        other.brownoutFollower = this;
        this.set(ControlMode.Follower, other.getChannel());
    }


    public void setupCurrentLimiting(int currentLimit, int currentLimitThreshold, int limitTimeout) {
        this.configContinuousCurrentLimit(currentLimit, 0);
        this.configPeakCurrentLimit(currentLimitThreshold, 0);
        this.configPeakCurrentDuration(limitTimeout, 0);
        this.enableCurrentLimit(true);
    }


    public void disableCurrentLimiting() {
        this.enableCurrentLimit(false);
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

    public void configPID(double P, double D, double F) {
        configPID(P, 0, D, F, 0);
    }

    public void configPID(double P, double I, double D, double F, int iZone) {
        // If these ever need to be nonzero, we can make them parameters instead
        final int pidSlot = 0, profileSlot = 0;

        configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidSlot, TIMEOUT_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT_MS);

        configNominalOutputForward(0, TIMEOUT_MS);
        configNominalOutputReverse(0, TIMEOUT_MS);
        configPeakOutputForward(1, TIMEOUT_MS);
        configPeakOutputReverse(-1, TIMEOUT_MS);

        // MARK - PIDF stuff
        selectProfileSlot(profileSlot, pidSlot);
        config_kF(profileSlot, F, TIMEOUT_MS);
        config_kP(profileSlot, P, TIMEOUT_MS);
        config_kI(profileSlot, I, TIMEOUT_MS);
        config_kD(profileSlot, D, TIMEOUT_MS);
        config_IntegralZone(profileSlot, iZone, TIMEOUT_MS);
    }

    public void postEstimatedKf(String name) {
        double speed = getSelectedSensorVelocity(0);
        if (Math.abs(speed) > 10) {
        }
    }
}