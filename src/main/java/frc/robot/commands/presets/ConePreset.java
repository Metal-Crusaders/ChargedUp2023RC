package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.claw.ClawPreset;
import frc.robot.commands.elevator.ElevatorPresetSpecific;
import frc.robot.commands.pivot.PivotPreset;
import frc.robot.subsystems.Claw;

public class ConePreset extends SequentialCommandGroup {

    private Pivot pivot;
    private Elevator elevator;
    private Claw claw;

    private PivotPreset pivotPreset;
    private ElevatorPresetSpecific elevatorPreset;
    private ClawPreset clawPreset;

    public ConePreset(Pivot pivot, Elevator elevator, Claw claw) {
        super();
        this.pivot = pivot;
        this.elevator = elevator;
        this.claw = claw;

        addRequirements(pivot);
        addRequirements(elevator);
        addRequirements(claw);

       pivotPreset = new PivotPreset(pivot, 320);
       elevatorPreset = new ElevatorPresetSpecific(elevator, 4940);
       clawPreset = new ClawPreset(claw, -447);

        addCommands(
            pivotPreset,
            clawPreset,
            elevatorPreset
        );
    }
}
