package frc.robot.commands.manipulatorCommands.ampCommands;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulators.AmpSubsystem;
import frc.robot.subsystems.sensors.Lighting;




public class ampCmd extends Command {
    
    private final AmpSubsystem ampSubsystem;
    private final double speed;
    private final Lighting leds;

    public ampCmd(AmpSubsystem ampSubsystem, double speed, Lighting leds) {
        this.ampSubsystem = ampSubsystem;
        this.speed = speed;
        this.leds = leds;
        addRequirements(ampSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ampCmd started");
    }

    @Override
    public void execute() {
        if (ampSubsystem.isBroken()){
            leds.orangePulse();
            ampSubsystem.spinAmp(speed);
        } else {
            ampSubsystem.spinAmp(speed);
            leds.green();
        }
    }

    @Override
    public void end(boolean interrupted) {
        ampSubsystem.spinAmp(0);
        System.out.println("ampCmd ended");
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
