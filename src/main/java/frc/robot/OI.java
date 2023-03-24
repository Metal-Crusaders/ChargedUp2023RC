package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {

    public static int XBOX_A = 1;
    public static int XBOX_B = 2;
    public static int XBOX_X = 3;
    public static int XBOX_Y = 4;
    public static int XBOX_LB = 5;
    public static int XBOX_RB = 6;

    private XboxController driver, operator;

    public OI() {
        this.driver = new XboxController(RobotMap.DRIVER_ID);
        this.operator = new XboxController(RobotMap.OPERATOR_ID);
    }

    public double getDriverXboxLeftY() {
        return -this.driver.getLeftY();
    }

    public double getDriverXboxLeftX() {
        return (-this.driver.getLeftX());
    }

    public double getDriverXboxRightY() {
        return this.driver.getRightY();
    }

    public double getDriverXboxRightX() {
        return this.driver.getRightX();
    }

    public double getDriverXboxLeftTrigger() {
        return this.driver.getLeftTriggerAxis();
    }

    public double getDriverXboxRightTrigger() {
        return this.driver.getRightTriggerAxis();
    }

    public XboxController getDriverXbox() {
        return this.driver;
    }

    public double getOperatorXboxLeftY() {
        return -this.operator.getLeftY();
    }

    public double getOperatorXboxLeftX() {
        return (-this.operator.getLeftX());
    }

    public double getOperatorXboxRightY() {
        return this.operator.getRightY();
    }

    public double getOperatorXboxRightX() {
        return this.operator.getRightX();
    }

    public double getOperatorXboxLeftTrigger() {
        return this.operator.getLeftTriggerAxis();
    }

    public double getOperatorXboxRightTrigger() {
        return this.operator.getRightTriggerAxis();
    }

    public XboxController getOperatorXbox() {
        return this.operator;
    }

}