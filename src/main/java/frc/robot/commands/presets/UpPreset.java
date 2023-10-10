package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.claw.ClawPreset;
import frc.robot.commands.elevator.ElevatorPreset;
import frc.robot.commands.pivot.PivotPreset;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class UpPreset extends CommandBase {

    private Pivot pivot;
    private Elevator elevator;
    private Claw claw;

    private double clawTicks = -591.000000, pivotTicks = 1100.500000;

    private double clawSpeed, pivotSpeed;

    private final double DEADBAND_CLAW = 5, DEADBAND_PIVOT = 5;
    private double WRIST_FULL_POWER = 0.5;
  
    double PIVOT_UP_FULL_POWER = 0.3;
    double PIVOT_DOWN_FULL_POWER = 0.1;

    public UpPreset(Pivot pivot, Elevator elevator, Claw claw) {
        super();
        this.pivot = pivot;
        this.elevator = elevator;
        this.claw = claw;

        addRequirements(pivot);
        addRequirements(elevator);
        addRequirements(claw);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // once everything else is done, do claw stuff
        if (!(Math.abs(claw.getWristTicks() - clawTicks) <= DEADBAND_CLAW)) {
            clawSpeed = (claw.getWristTicks() - clawTicks) / (Math.abs(claw.getWristTicks() - clawTicks)) * WRIST_FULL_POWER;
            SmartDashboard.putNumber("Claw Speed Defined in Claw Preset", clawSpeed);
            claw.setWrist(clawSpeed);
        } else {
            claw.setWrist(0);
        }

        if (!(Math.abs(pivot.getEncoderTicks() - pivotTicks) <= DEADBAND_PIVOT)) {
            pivotSpeed = (pivotTicks - pivot.getEncoderTicks()) / (Math.abs(pivotTicks - pivot.getEncoderTicks())); // (pivot.getEncoderTicks() - pivotTicks) * kPPivot * 
            if (pivotSpeed > 0) {
            pivotSpeed *= PIVOT_UP_FULL_POWER;
            } else {
            pivotSpeed *= PIVOT_DOWN_FULL_POWER;
            }
            pivot.set(pivotSpeed);
        } else {
            pivot.stop();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        claw.setWrist(0);
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return (
            (Math.abs(claw.getWristTicks() - clawTicks) <= DEADBAND_CLAW) &&
            (Math.abs(pivot.getEncoderTicks() - pivotTicks) <= DEADBAND_PIVOT)
        );
    }

}


// public class UpPreset extends ParallelCommandGroup {

//     private Pivot pivot;
//     private Elevator elevator;
//     private Claw claw;

//     private PivotPreset pivotPreset;
//     private ElevatorPreset elevatorPreset;
//     private ClawPreset clawPreset;

//     public UpPreset(Pivot pivot, Elevator elevator, Claw claw) {
//         super();
//         this.pivot = pivot;
//         this.elevator = elevator;
//         this.claw = claw;

//         addRequirements(pivot);
//         addRequirements(elevator);
//         addRequirements(claw);

//        pivotPreset = new PivotPreset(pivot, 1100.500000);
//        clawPreset = new ClawPreset(claw, -591.000000);

//         addCommands(
//             pivotPreset,
//             clawPreset
//         );
//     }
// }
