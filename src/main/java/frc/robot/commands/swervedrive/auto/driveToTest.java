package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class driveToTest extends Command{
    private final SwerveSubsystem swerve;
    private final PIDController controller;
    
    public driveToTest(SwerveSubsystem swerve, Pose2d pose)
    {
        this.swerve = swerve;
        controller = new PIDController(1.0, 0.0, 0.0);
        controller.setTolerance(1);
        controller.setSetpoint(0.0);
        addRequirements(swerve);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
      return false;
    }
}
