package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.tools.DriveStraightAuto;
import frc.robot.commands.autonomous.tools.DriveTurnAuto;
import frc.robot.subsystems.TankDrive;

public class LeaveAndChargeAuto extends SequentialCommandGroup {

    private double DRIVE_STRAIGHT_TARGET = 32135.870117; // TODO change this

    private TankDrive drive;

    private DriveStraightAuto driveStraightAuto;
    private DriveTurnAuto turn180Auto;
    private FastChargeAuto fastChargeAuto;

    public LeaveAndChargeAuto(TankDrive drive) {
        super();
        this.drive = drive;

        addRequirements(drive);

        driveStraightAuto = new DriveStraightAuto(drive, DRIVE_STRAIGHT_TARGET);
        turn180Auto = new DriveTurnAuto(drive, 155);
        fastChargeAuto = new FastChargeAuto(drive);

        addCommands(
                driveStraightAuto,
                turn180Auto,
                fastChargeAuto
        );
    }

}
