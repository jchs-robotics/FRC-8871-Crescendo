package frc.robot.commands.manipulatorCommands.pivotCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulators.PivotSubsystem;

public class rotatePIDCmdUp extends Command {

    private final PivotSubsystem PiSs;
    private final PIDController pidController;
    private double point;
    
    public rotatePIDCmdUp(PivotSubsystem PiSs, double setpoint) {
        this.PiSs = PiSs;
        // one motor p: 0.35 i:0 d:0.012
        this.pidController = new PIDController(0.28, 0.0, 0.009);
        pidController.setSetpoint(setpoint);
        point = setpoint;
        addRequirements(PiSs);
    }

    @Override
    public void initialize() {
        System.out.println("pivotPIDCmdUp started");
        pidController.reset();
    }

    @Override
    public void execute() {
        
        double speed = pidController.calculate(PiSs.pivotEncTwoBI.getPosition());
       // double speed = pidController.calculate(PiSs.tbEncoder.getAbsolutePosition());

        PiSs.setMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        PiSs.setMotors(0);
        System.out.println("pivotPIDCmd ended");
    }

    @Override
    public boolean isFinished() {
        if (PiSs.pivotEncTwoBI.getPosition() <= point)
        {
            return true;
        } else {
            return false;
        }
        // if (PiSs.tbEncoder.getAbsolutePosition() >= point)
        // {
        //     return true;
        // } else {
        //     return false;
        // }
    }
}
