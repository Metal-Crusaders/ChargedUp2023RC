package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TankDrive;

public class ForwardBackward extends SequentialCommandGroup {

    private final TankDrive drive;

    private double targetF, targetB;

    private DriveStraightAuto driveForward, driveBackward;

    public ForwardBackward(TankDrive drive, double targetForwards, double targetBackwards) {
        super();
        this.drive = drive;
        this.targetF = targetForwards;
        this.targetB = -1 * targetBackwards;

        addRequirements(drive);

        driveForward = new DriveStraightAuto(this.drive, this.targetF);
        driveBackward = new DriveStraightAuto(this.drive, this.targetB);

        addCommands(
                driveForward,
                driveBackward
        );
    }



}