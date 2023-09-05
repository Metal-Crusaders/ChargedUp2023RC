package frc.robot.commands.autonomous.tools;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.tools.DriveStraightAuto;
import frc.robot.commands.autonomous.tools.DriveTurnAuto;
import frc.robot.commands.autonomous.tools.HighCubeShot;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TankDrive;

public class ShootAndTurnAuto extends SequentialCommandGroup {

    private TankDrive drive;
    private Claw claw;

    private HighCubeShot shootCube;
    private BFDriveStraightAuto littleBack;
    private DriveTurnAuto turn180;


    public ShootAndTurnAuto(TankDrive drive, Claw claw) {
        super();
        this.drive = drive;
        this.claw = claw;

        addRequirements(this.drive);
        addRequirements(this.claw);

        shootCube = new HighCubeShot(claw);
        littleBack = new BFDriveStraightAuto(drive, 0.5, 0.25);
        turn180 = new DriveTurnAuto(drive, 160);

        addCommands(
                shootCube,
                turn180
        );
    }

}
