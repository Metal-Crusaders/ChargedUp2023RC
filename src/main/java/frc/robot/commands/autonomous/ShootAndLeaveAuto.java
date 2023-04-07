package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.tools.*;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Claw;

public class ShootAndLeaveAuto extends SequentialCommandGroup {

    private TankDrive drive;
    private Claw claw;

    private ShootAndTurnAuto shootAndTurnAuto;
    private DriveStraightAuto leaveCommunity;


    public ShootAndLeaveAuto(TankDrive drive, Claw claw) {
        super();
        this.drive = drive;
        this.claw = claw;

        addRequirements(this.drive);
        addRequirements(this.claw);

        shootAndTurnAuto = new ShootAndTurnAuto(this.drive, this.claw);
        leaveCommunity = new DriveStraightAuto(this.drive, 30500);

        addCommands(
                shootAndTurnAuto,
                leaveCommunity
        );
    }

}
