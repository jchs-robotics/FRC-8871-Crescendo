package frc.robot.commands.manipulatorCommands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulators.IntakeSubsystem;

public class intakeManCmd extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final double speed;
    
    public intakeManCmd(IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("intake extend started");
    }

    @Override
    public void execute() {
        intakeSubsystem.extendIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.extendIntake(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
