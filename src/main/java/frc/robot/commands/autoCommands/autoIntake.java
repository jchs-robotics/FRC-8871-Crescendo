package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulators.ShooterSubsystem;
import frc.robot.commands.manipulatorCommands.shooterCmd;
import frc.robot.commands.manipulatorCommands.intakeCommands.intakeCmdExtend;
import frc.robot.commands.manipulatorCommands.intakeCommands.spinIntakeCmd;
import frc.robot.commands.manipulatorCommands.pivotCommands.rotatePIDCmdDown;
import frc.robot.subsystems.manipulators.IntakeSubsystem;
import frc.robot.subsystems.manipulators.PivotSubsystem;


public class autoIntake extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final PivotSubsystem pivotSubsystem;

    private final Timer runTime;

    private double initialTime;
    private double currentTime;

    public autoIntake(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        runTime = new Timer();
        addRequirements(shooterSubsystem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        new SequentialCommandGroup(
      // extend intake
        new intakeCmdExtend(intakeSubsystem, 41.5),

      // lower pivot
        new rotatePIDCmdDown(pivotSubsystem, 15.3),

      // spin shooter (reverse) + spin intake
        new ParallelCommandGroup(
            new shooterCmd(shooterSubsystem, -1),
            new spinIntakeCmd(intakeSubsystem, 0.65) 
        )
    );
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
