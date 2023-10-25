package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.tools.BalanceAuto;
import frc.robot.commands.autonomous.tools.DriveStraightAuto;
import frc.robot.commands.autonomous.tools.DriveTurnAuto;
import frc.robot.subsystems.TankDrive;

public class LeaveAndChargeAuto extends SequentialCommandGroup {

    private double DRIVE_STRAIGHT_TARGET = 22000; // TODO change this

    private TankDrive drive;

    private DriveStraightAuto driveStraightAuto;
    private DriveTurnAuto turn180Auto;
    private ChargePanelAuto chargePanelAuto;

    public LeaveAndChargeAuto(TankDrive drive) {
        super();
        this.drive = drive;

        addRequirements(drive);

        driveStraightAuto = new DriveStraightAuto(drive, DRIVE_STRAIGHT_TARGET, 1000);
        turn180Auto = new DriveTurnAuto(drive, 160);
        chargePanelAuto = new ChargePanelAuto(drive, false);

        addCommands(
                driveStraightAuto,
                turn180Auto,
                chargePanelAuto
        );
    }

}
