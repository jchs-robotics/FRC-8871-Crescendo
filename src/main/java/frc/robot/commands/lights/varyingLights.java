package frc.robot.commands.lights;
import frc.robot.subsystems.sensors.Lighting;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;

public class varyingLights extends Command{
    private final Lighting leds;
    private int brightness = 255;

    public varyingLights(Lighting leds)
    {
        this.leds = leds;
        //brightness = 255;
        addRequirements(leds);
    }

    @Override
    public void initialize()
    {
        leds.startLighting();
        leds.setlengthHSV(1, 144, 30, 255, brightness);
        System.out.println("BITCH multiinitialize");
    }

    @Override
    public void execute()
    {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        if (brightness != 0)
        {
        brightness -= 5;
        leds.setlengthHSV(1, 144, 30, 255, brightness);
        } else {
        brightness += 5;
        leds.setlengthHSV(1, 144, 30, 255, brightness);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        leds.setlengthHSV(1, 144, 0, 0, 0);
        leds.stop();
        System.out.println("BITCH multibye");
    }

    @Override
    public boolean isFinished()
    {
      return false;
    }

}
