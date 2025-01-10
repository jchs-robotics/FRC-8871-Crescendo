package frc.robot.commands.manipulatorCommands.elevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulators.ElevatorSubsystem;

public class elevatorManCmd extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;
    
    public elevatorManCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("elevator started");
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevator(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevator(0);

    }

    @Override // FIXME does this mess up the other commands?
    public boolean isFinished() {
        return false;

        // hard stop && trigger held...
        // if (elevatorSubsystem.elevatorEncOne.getPosition() <= -245) {
        //     return true;
        // } else if  (elevatorSubsystem.elevatorEncOne.getPosition() >= -2) {
        //     return true;
        // } else {
        //     return false;
        // }
    }
}
