package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.claw.ClawPreset;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.pivot.PivotPreset;
import frc.robot.subsystems.Claw;

public class DefaultPreset extends SequentialCommandGroup {

    private Pivot pivot;
    private Elevator elevator;
    private Claw claw;

    private PivotPreset pivotPreset;
    private ElevatorPreset elevatorPreset;
    private ClawPreset clawPreset;

    public DefaultPreset(Pivot pivot, Elevator elevator, Claw claw) {
        super();
        this.pivot = pivot;
        this.elevator = elevator;
        this.claw = claw;

        addRequirements(pivot);
        addRequirements(elevator);
        addRequirements(claw);

       pivotPreset = new PivotPreset(pivot, 0);
       elevatorPreset = new ElevatorPreset(elevator, false);
       clawPreset = new ClawPreset(claw, 0, false);

        addCommands(
            elevatorPreset,
            clawPreset,
            pivotPreset
        );
    }
}
