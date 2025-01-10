package frc.robot.commands.lights;
import frc.robot.subsystems.sensors.Lighting;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.IntSupplier;

public class offLights extends Command{
    private final Lighting leds;

    public offLights(Lighting leds)
    {
        this.leds = leds;
    }

    @Override
    public void initialize()
    {
        leds.startLighting();
        System.out.println("BITCH HALFinitialize");
    }

    @Override
    public void execute()
    {
        leds.setSolidColor(230, 50, 250);;
        System.out.println("BITCH HALFexecute");
    }

    @Override
    public void end(boolean interrupted)
    {
        leds.setSolidColor(0, 0, 0);
        leds.stop();
        System.out.println("BITCH HALFbye");
    }

    @Override
    public boolean isFinished()
    {
      return false;
    }

}
