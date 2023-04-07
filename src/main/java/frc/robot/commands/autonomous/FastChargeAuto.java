package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.tools.BalanceAuto;
import frc.robot.commands.autonomous.tools.BFDriveStraightAuto;
import frc.robot.commands.autonomous.tools.MountPanelAuto;
import frc.robot.subsystems.TankDrive;

public class FastChargeAuto extends SequentialCommandGroup {

    private TankDrive drive;

    private MountPanelAuto mountPanelAuto;
    private BFDriveStraightAuto estimateMidAuto;
    private BalanceAuto balanceAuto;

    public FastChargeAuto(TankDrive drive) {
        super();
        this.drive = drive;

        addRequirements(drive);

        mountPanelAuto = new MountPanelAuto(drive, false);
        estimateMidAuto = new BFDriveStraightAuto(drive, 2, 0.4); // TODO TWEAK THIS
        balanceAuto = new BalanceAuto(drive, false);

        addCommands(
                mountPanelAuto,
                estimateMidAuto,
                balanceAuto
        );
    }

}
