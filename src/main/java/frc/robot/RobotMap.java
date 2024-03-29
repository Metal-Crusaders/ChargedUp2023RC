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
    public static final int REV_PH_ID = 10;

    public static final boolean LEFT_DT_INVERTED = false;
    public static final boolean LEFT_PIV_INVERTED = false;
    public static final boolean ELEVATOR_REVERSED = true;
    public static final boolean CLAW_REVERSED = false;
    public static final boolean WRIST_REVERSED = true;

    // PWM / DIO
    public static final int LEFT_PIVOT = 5;
    public static final int RIGHT_PIVOT = 6;
    public static final int ELEVATOR_CAN_ID = 5; // TODO change this if necessary
    public static final int PIVOT_ENCODER_IN = 6;
    public static final int PIVOT_ENCODER_OUT = 7;
    public static final int CLAW_ROLLER1 = 1;
    public static final int CLAW_ROLLER2 = 2;
    public static final int CLAW_WRIST = 9;
    public static final int WRIST_ENCODER_IN = 2;
    public static final int WRIST_ENCODER_OUT = 3;

    // PNEUMATICS
    public static final int CLAW_IN = 6;
    public static final int CLAW_OUT = 7;

}
