package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.tools.BalanceAuto;
import frc.robot.commands.autonomous.tools.DriveStraightAuto;
import frc.robot.commands.autonomous.tools.MountPanelAuto;
import frc.robot.subsystems.TankDrive;

public class FastChargeAuto extends SequentialCommandGroup {

    private double TARGET = 15000;

    private TankDrive drive;

    private DriveStraightAuto mountPanelAuto;
    private BalanceAuto balanceAuto;

    public FastChargeAuto(TankDrive drive) {
        super();
        this.drive = drive;

        addRequirements(drive);

        mountPanelAuto = new DriveStraightAuto(drive, TARGET);
        balanceAuto = new BalanceAuto(drive, false);

        addCommands(
                mountPanelAuto,
                balanceAuto
        );
    }

}
