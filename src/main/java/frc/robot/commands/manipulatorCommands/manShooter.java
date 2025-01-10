package frc.robot.commands.manipulatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulators.ShooterSubsystem;

public class manShooter extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier speed;

    public manShooter(ShooterSubsystem shooterSubsystem, DoubleSupplier speed) {
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
        double spinSpeed = speed.getAsDouble();
        shooterSubsystem.setShooter(spinSpeed);
    }
 
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooter(0);
        System.out.println("shooterCmd ended");
    }

    @Override
    public boolean isFinished() {
        if (speed.getAsDouble() == 0) {
            return true;
        } else {
            return false;
        }
    }
}
