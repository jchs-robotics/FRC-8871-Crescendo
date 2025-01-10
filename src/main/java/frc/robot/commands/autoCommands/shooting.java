package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulators.ShooterSubsystem;


// runs shooter for x seconds

public class shooting extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private final Timer shootingTime;

    private double initialTime;
    private double currentTime;

    public shooting(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        shootingTime = new Timer();
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {
        shootingTime.reset();
        // this.initialTime = shootingTime.getFPGATimestamp();
        System.out.println("ampCmd started");
    }

    @Override
    public void execute() {
        currentTime =  Timer.getFPGATimestamp();
        if (currentTime < 5){
            shooterSubsystem.setShooter(1);
        }else{
            shooterSubsystem.setShooter(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooter(0);
    }

    @Override
    public boolean isFinished() {
        return currentTime > 5;
    }
}
