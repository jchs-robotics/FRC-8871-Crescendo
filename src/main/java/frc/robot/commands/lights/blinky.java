package frc.robot.commands.lights;
import frc.robot.subsystems.sensors.Lighting;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.IntSupplier;

public class blinky extends Command{
    private final Lighting leds;

    public blinky(Lighting leds)
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
        leds.breathe();
        leds.setData();
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
