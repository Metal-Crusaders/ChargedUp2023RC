package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.presets.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.autonomous.tools.*;

public class EdgeAuto extends SequentialCommandGroup {

    private TankDrive drive;
    private Pivot pivot;
    private Elevator elevator;
    private Claw claw;

    public EdgeAuto(TankDrive drive, Pivot pivot, Elevator elevator, Claw claw) {
        super();
        this.drive = drive;
        this.pivot = pivot;
        this.elevator = elevator;
        this.claw = claw;

        addRequirements(drive);
        addRequirements(pivot);
        addRequirements(elevator);
        addRequirements(claw);

        UpPreset upPreset = new UpPreset(pivot, elevator, claw);
        RollerAuto spit = new RollerAuto(claw, 0.1, true);
        DriveStraightAuto backup = new DriveStraightAuto(drive, -2174.997437, 50);
        DefaultPreset defaultPreset = new DefaultPreset(pivot, elevator, claw);
        DriveTurnAuto turn1 = new DriveTurnAuto(drive, 155);
        CubePreset cubePreset = new CubePreset(pivot, elevator, claw);

        addCommands(
            upPreset,
            spit,
            new ParallelCommandGroup(backup, defaultPreset),
            turn1,
            new DriveStraightAuto(drive, 4000, 50), new DriveTurnAuto(drive, 15),
            new ParallelCommandGroup(cubePreset, new DriveStraightAuto(drive, 20000, 1000))
        );
    }

}
