package frc.robot.commands.manipulatorCommands.pivotCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulators.PivotSubsystem;

public class rotatePIDCmdDown extends Command {

    private final PivotSubsystem PiSs;
    private final PIDController pidController;
    private double point;
    

    // TODO add derivative value
    public rotatePIDCmdDown(PivotSubsystem PiSs, double setpoint) {
        this.PiSs = PiSs;
        this.pidController = new PIDController(0.1, 0.0, 0.009);
        pidController.setSetpoint(setpoint);
        point = setpoint;
        addRequirements(PiSs);
    }

    @Override
    public void initialize() {
        System.out.println("pivotPIDCmdDown started");
        pidController.reset();
    }

    @Override
    public void execute() {
        //double speed = pidController.calculate(PiSs.pivotEncOne.getAbsolutePosition());
        double speed = pidController.calculate(PiSs.pivotEncTwoBI.getPosition());
        PiSs.setMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        PiSs.setMotors(0);
        System.out.println("pivotPIDCmd ended");
    }

    @Override
    public boolean isFinished() {
        if (PiSs.pivotEncTwoBI.getPosition() >= (point - 0.15))
        {
        return true;
        } else {
            return false;
        }
    }
}
