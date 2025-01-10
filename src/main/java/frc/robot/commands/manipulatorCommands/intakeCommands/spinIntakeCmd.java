package frc.robot.commands.manipulatorCommands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulators.IntakeSubsystem;


// FIXME Can this subsystem be added to the intakecmd pid command?


public class spinIntakeCmd extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;
    private final double moreSpeed;

    public spinIntakeCmd(IntakeSubsystem intakeSubsystem, double speed) {
        moreSpeed = speed;
        
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("spinIntakeCmd started");
       // intakeSubsystem.coast();
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntake(0);
        System.out.println("spinIntakeCmd ended");
    }

    @Override
    public boolean isFinished() {
        if (moreSpeed == 0) {
            return true;
        } else {
            return false;
        }
    }
}
