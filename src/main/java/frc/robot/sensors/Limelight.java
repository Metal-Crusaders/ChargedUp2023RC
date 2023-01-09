/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Limelight {

    // Hardware constants go below


    public double xError;
    public double yError;
    public double targetArea;
    public long targetFound;

    public Limelight() {}

    public void periodic() {

        // Data grabbing
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        // Read data
        xError = tx.getDouble(0);
        yError = ty.getDouble(0);
        targetArea = ta.getDouble(0);
        targetFound = tv.getInteger(0);

        // Project data to SmartDashboard
        SmartDashboard.putNumber("X angle from target", xError);
        SmartDashboard.putNumber("Y angle from target", yError);
    }

}
