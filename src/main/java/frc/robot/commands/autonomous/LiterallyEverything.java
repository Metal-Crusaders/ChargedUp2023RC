package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.tools.ShootAndTurnAuto;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TankDrive;

public class LiterallyEverything extends SequentialCommandGroup {

    private TankDrive drive;
    private Claw claw;

    private ShootAndTurnAuto shootAndTurnAuto;
    private LeaveAndChargeAuto leaveAndChargeAuto;

    public LiterallyEverything(TankDrive drive, Claw claw) {
        super();
        this.drive = drive;
        this.claw = claw;

        addRequirements(this.drive);
        addRequirements(this.claw);

        shootAndTurnAuto = new ShootAndTurnAuto(this.drive, this.claw);
        leaveAndChargeAuto = new LeaveAndChargeAuto(this.drive);

        addCommands(
                shootAndTurnAuto,
                leaveAndChargeAuto
        );
    }

}
