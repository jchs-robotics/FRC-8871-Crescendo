package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.manipulatorCommands.elevatorCommands.elevatorPIDCmdExtend;
import frc.robot.commands.manipulatorCommands.pivotCommands.rotatePIDCmdUp;
import frc.robot.subsystems.manipulators.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.PivotSubsystem;
import frc.robot.subsystems.manipulators.ShooterSubsystem;



// FIXME FIXME URGENT FIXME 
// no pid rn

//error to get attention

public class aimPivot extends SequentialCommandGroup {
     public aimPivot(PivotSubsystem pivotSubsystem) {
            new rotatePIDCmdUp(pivotSubsystem, -27);
            //new rotatePIDCmdUp(pivotSubsystem, -27);
    }
}
