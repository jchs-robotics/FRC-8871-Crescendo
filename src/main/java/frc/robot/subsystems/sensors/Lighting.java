package frc.robot.subsystems.sensors;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants;
import frc.robot.commands.lights.testHalfLights;

public class Lighting extends SubsystemBase{

    private final AddressableLED m_lighting = new AddressableLED(0);
    private AddressableLEDBuffer m_AddressableLEDBuffer = new AddressableLEDBuffer(28);
    private int m_rainbowFirstPixelHue;


    private int redPulseBrightness = 0;
    private Boolean redCycle = true;

    private int orangePulseBrightness = 0;
    private Boolean orangeCycle = true;

    public Lighting()
    {
        m_lighting.setLength(m_AddressableLEDBuffer.getLength());
        m_lighting.setData(m_AddressableLEDBuffer);
        startLighting();
    }
    
    @Override
    public void periodic() {

    }

    public void startLighting()
    { 
        m_lighting.start();
    }

    public void setSolidColor(int red, int green, int blue)
    {
        for (var i = 0; i < getLength(); i++) 
        {
            setRGB(i, red, green, blue);
        }
        setData();
    }
    //for rgb length
    public void setlength(int start, int end, int red, int green, int blue)
    {
        for (var i = start; i <= end; i++) 
        {
            setRGB(i, red, green, blue);
        }
        setData();
    }

    public void setlengthHSV(int start, int end, int h, int s, int v)
    {
        for (var i = start; i <= end; i++) 
        {
            setHSV(i, h, s, v);
        }
        setData();
    }

    public int getLength()
    {
        return m_AddressableLEDBuffer.getLength();
    }

    public void setRGB(int i, int r, int g, int b){
        m_AddressableLEDBuffer.setRGB(i, r, g, b);
    }

    public void setData()
    {
        m_lighting.setData(m_AddressableLEDBuffer);
    }
    
    public void stop()
    {
        m_lighting.stop();
    }

    public void setHSV(int i, int h, int s, int v)
    {
        m_AddressableLEDBuffer.setHSV(i, h, s, v);
    }
    
    public Command rainbow() {
        // For every pixel
        return run (()-> {
        for (var i = 0; i < getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_AddressableLEDBuffer.getLength())) % 180;
          // Set the value
          m_AddressableLEDBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        setData();
      });
    }

    public Command breathe() {
        return run(()-> {
        // For every pixel
        for (var i = 0; i < getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_AddressableLEDBuffer.getLength())) % 180;
          // Set the value
          m_AddressableLEDBuffer.setHSV(i, 30, 255, hue);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        setData();
      });
    }


    //FIXME change to commands
    public Command red() {
        return run(()-> {
        for (var i = 0; i < m_AddressableLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_AddressableLEDBuffer.setRGB(i, 255, 0, 0);
        }
        
        m_lighting.setData(m_AddressableLEDBuffer);
        });
        }
    
        public Command blue() {
        return run(()-> {
        for (var i = 0; i < m_AddressableLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for blue
            m_AddressableLEDBuffer.setRGB(i, 0, 0, 255);
        }
        
        m_lighting.setData(m_AddressableLEDBuffer);
        });
    }
    
        public Command green() {
        return run(()-> {
        for (var i = 0; i < m_AddressableLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for green
            m_AddressableLEDBuffer.setRGB(i, 0, 255, 0);
        }
        
        m_lighting.setData(m_AddressableLEDBuffer);
    });
}
    
        public Command purple() {
            return run(()-> {
        for (var i = 0; i < m_AddressableLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for purple
            m_AddressableLEDBuffer.setRGB(i, 148, 0, 211);
        }
        
        m_lighting.setData(m_AddressableLEDBuffer);
        }
        );} 
    
        public Command redPulse(){ //disabled pattern
        return run(()-> {
        for (var i = 0; i < m_AddressableLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for blue pulse
            m_AddressableLEDBuffer.setRGB(i, redPulseBrightness*5, 0, 0);
        }
        //increase brightness
        if ((redPulseBrightness == 51) || (redPulseBrightness == 0)) {
            redCycle = !(redCycle);
        }
        if ((redCycle) && (redPulseBrightness < 51)) {
            redPulseBrightness += 1;
        }
        if (!(redCycle) &&  (redPulseBrightness > 0)) {
            redPulseBrightness -= 1;
        }
        m_lighting.setData(m_AddressableLEDBuffer);

    }
    );}    

        public Command orangePulse(){ //code for when it's enabled  pattern
        return run(()-> {
        for (var i = 0; i < m_AddressableLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for blue pulse
            m_AddressableLEDBuffer.setHSV(i, orangePulseBrightness*5, 100, 20);
        }
        //increase brightness
        if ((orangePulseBrightness == 51) || (orangePulseBrightness == 0)) {
            orangeCycle = !(orangeCycle);
        }
        if ((orangeCycle) && (orangePulseBrightness < 51)) {
            orangePulseBrightness += 1;
        }
        if (!(redCycle) &&  (orangePulseBrightness > 0)) {
            orangePulseBrightness -= 1;
        }
        m_lighting.setData(m_AddressableLEDBuffer);

    }
    );}    





    }