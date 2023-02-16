package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TankDrive;

public class SwivelTest extends SequentialCommandGroup {

    private final TankDrive drive;

    private double target1, target2;

    private DriveTurnAuto turnLeft, turnRight;

    public SwivelTest(TankDrive drive, double target1, double target2) {
        super();
        this.drive = drive;
        this.target1 = target1;
        this.target2 = -1 * target2;

        addRequirements(drive);

        turnLeft = new DriveTurnAuto(drive, this.target1);
        turnRight = new DriveTurnAuto(drive, this.target2);

        addCommands(
                turnLeft,
                turnRight
        );
    }



}
