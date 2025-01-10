package frc.robot.commands.lights;
import frc.robot.subsystems.sensors.Lighting;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.IntSupplier;

public class defaultLights extends Command{
    private final Lighting leds;

    public defaultLights(Lighting leds)
    {
        this.leds = leds;

        addRequirements(leds);
    }

    @Override
    public void initialize()
    {
        leds.startLighting();
    }

    @Override
    public void execute()
    {
        leds.setSolidColor(250, 50, 5);
    }

    @Override
    public void end(boolean interrupted)
    {
        leds.setSolidColor(0, 0, 0);
        leds.stop();
        System.out.println("BITCHdefbye");
    }

    @Override
    public boolean isFinished()
    {
      return false;
    }
}
