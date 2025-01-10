package frc.robot.commands.manipulatorCommands.elevatorCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulators.ElevatorSubsystem;

public class elevatorPIDCmdExtend extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final PIDController pidController;
    private final double point;
    
    // FIXME could the low p-value be the reason why the elevator is so slow? or is it mechanical?
    public elevatorPIDCmdExtend(ElevatorSubsystem elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        point = setpoint;
        this.pidController = new PIDController(0.7, 0.01, 0.0);
        pidController.setSetpoint(setpoint);
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("elevator started");
        pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(elevatorSubsystem.elevatorEncOne.getPosition());
        elevatorSubsystem.setElevator(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevator(0);
        System.out.println("elevatorPIDCmd ended");
    }

    @Override
    public boolean isFinished() {
        if (elevatorSubsystem.elevatorEncOne.getPosition() <= point)
        {
        return true;
        } else {
            return false;
        }
    }
}
