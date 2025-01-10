package frc.robot.commands.manipulatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulators.ShooterSubsystem;

public class shooterCmd extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private final double speed;

    public shooterCmd(ShooterSubsystem shooterSubsystem, double speed) {
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("shooterCmd started");
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooter(speed);
    }
 
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooter(0);
        System.out.println("shooterCmd ended");
    }

    @Override
    public boolean isFinished() {
        if (speed == 0) {
            return true;
        } else {
            return false;
        }
    }
}
