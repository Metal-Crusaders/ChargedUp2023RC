// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    // OI Stuff
    public static final int DRIVER_ID = 0;
    public static final int OPERATOR_ID = 1;

    // CAN STUFF
    public static final int LEFT_FRONT = 1;
    public static final int RIGHT_FRONT = 2;
    public static final int LEFT_REAR = 4;
    public static final int RIGHT_REAR = 3;
    public static final int LEFT_PIVOT = 5;
    public static final int RIGHT_PIVOT = 6;
    public static final int REV_PH_ID = 10;

    public static final boolean LEFT_DT_INVERTED = false;
    public static final boolean LEFT_PIV_INVERTED = false;
    public static final boolean ELEVATOR_REVERSED = false;
    public static final boolean CLAW_REVERSED = false;

    // PWM / DIO
    public static final int ELEVATOR_PWM_ID1 = 7;
    public static final int ELEVATOR_PWM_ID2 = 8;
    public static final int ELEVATOR_LOWER = 2; // TODO change these for limit switch
    public static final int ELEVATOR_UPPER = 3;

    public static final int CLAW_ROLLER1 = 5;
    public static final int CLAW_ROLLER2 = 6;

    // PNEUMATICS
    public static final int CLAW_IN = 1;
    public static final int CLAW_OUT = 2;

}
