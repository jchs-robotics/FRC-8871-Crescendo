package frc.robot.commands.manipulatorCommands.intakeCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulators.IntakeSubsystem;

public class intakeCmdRetract extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private PIDController pidController;
    private double setpoint;
    
    public intakeCmdRetract(IntakeSubsystem intakeSubsystem, double setpoint) {

        this.intakeSubsystem = intakeSubsystem;
        this.pidController = new PIDController(0.09, 0.00, 0.01);
        pidController.setSetpoint(setpoint);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

        System.out.println("intakeCmdRetract started");
        pidController.reset();
    }

    @Override
    public void execute() {

        double speed = pidController.calculate(intakeSubsystem.intakeEncoder.getPosition());
        intakeSubsystem.extendIntake(speed);
        // intakeSubsystem.setIntake(1);
    }

    @Override
    public void end(boolean interrupted) {

        intakeSubsystem.extendIntake(0);
       // intakeSubsystem.setIntake(0);
        System.out.println("intakeCmd ended");
    }

    @Override
    public boolean isFinished() {
        if (intakeSubsystem.intakeEncoder.getPosition() <= setpoint)
        {
            return true;
        } else {
            return false;
        }
    }
}
