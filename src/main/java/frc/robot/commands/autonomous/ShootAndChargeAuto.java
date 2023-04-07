package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.tools.DriveStraightAuto;
import frc.robot.commands.autonomous.tools.MountPanelAuto;
import frc.robot.commands.autonomous.tools.ShootAndTurnAuto;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TankDrive;

public class ShootAndChargeAuto extends SequentialCommandGroup {

    private TankDrive drive;
    private Claw claw;

    private ShootAndTurnAuto shootAndTurnAuto;
    private ChargePanelAuto chargePanelAuto;


    public ShootAndChargeAuto(TankDrive drive, Claw claw) {
        super();
        this.drive = drive;
        this.claw = claw;

        addRequirements(this.drive);
        addRequirements(this.claw);

        shootAndTurnAuto = new ShootAndTurnAuto(this.drive, this.claw);
        chargePanelAuto = new ChargePanelAuto(this.drive, false);

        addCommands(
                shootAndTurnAuto,
                chargePanelAuto
        );
    }

}
