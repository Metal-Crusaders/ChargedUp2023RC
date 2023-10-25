package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.tools.BalanceAuto;
import frc.robot.commands.autonomous.tools.MountPanelAuto;
import frc.robot.commands.autonomous.tools.DriveStraightAuto;
import frc.robot.subsystems.TankDrive;

public class ChargePanelAuto extends SequentialCommandGroup {

    private TankDrive drive;

    private MountPanelAuto mountPanelAuto;
    private BalanceAuto balanceAuto;


    public ChargePanelAuto(TankDrive drive, boolean isBackwards) {
        super();
        this.drive = drive;

        addRequirements(drive);

        mountPanelAuto = new MountPanelAuto(drive, isBackwards);
        balanceAuto = new BalanceAuto(drive, isBackwards);

        addCommands(
                mountPanelAuto,
                new DriveStraightAuto(drive, 9000, 250),
                balanceAuto
        );
    }

}
