package frc.robot.commands.manipulatorCommands.pivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulators.PivotSubsystem;

public class rotatePivotCmd extends Command {
    
    private final PivotSubsystem PiSs;
    private final double speed;

    public rotatePivotCmd(PivotSubsystem PiSs, double speed) {
        this.PiSs = PiSs;
        this.speed = speed;
        addRequirements(PiSs);
    }

    @Override
    public void initialize() {
        System.out.println("rotatePivotCmd started");
    }

    @Override
    public void execute() {
        PiSs.setMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        PiSs.setMotors(0);
        System.out.println("rotatePivotCmd ended");
    }

    @Override // FIXME messes up other pivot cmds?
    public boolean isFinished() {
        return false;
        // if ((PiSs.pivotEncTwoBI.getPosition() <= -50) || (PiSs.pivotEncTwoBI.getPosition() >= 16)) {
        //     return true;
        // } else {
        //     return false;
        // }
    }
}
