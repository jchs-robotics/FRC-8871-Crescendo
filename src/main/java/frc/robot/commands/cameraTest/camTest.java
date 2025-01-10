package frc.robot.commands.cameraTest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.lights.offLights;
import frc.robot.commands.lights.testHalfLights;
import frc.robot.subsystems.sensors.Lighting;
import frc.robot.subsystems.sensors.Limelight;


public class camTest extends Command{
    private final Lighting leds;
    private final Limelight limelight;

    public camTest(Limelight limelight, Lighting leds)
    {
      this.leds = leds;
      this.limelight = limelight;

      addRequirements(limelight);
    }

    @Override
    public void initialize()
    {
      leds.startLighting();
    }
    @Override
    public void execute()
    {
      if (limelight.getID() > 0.0)
      {
        leds.run((Runnable) new testHalfLights(leds));
        System.out.println("hi");
      } else {
        System.out.println("nawb");
        leds.run((Runnable) new offLights(leds));
      }
    }

    public void end()
    {
        leds.setSolidColor(0, 0, 0);
        leds.stop();
    }

    @Override
    public boolean isFinished()
    {
      return false;
    }
}
