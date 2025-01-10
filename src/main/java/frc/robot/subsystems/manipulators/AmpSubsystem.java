package frc.robot.subsystems.manipulators;
import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class AmpSubsystem extends SubsystemBase {

    // extend intake out
    private final CANSparkMax AmpMotor = new CANSparkMax(Constants.ManipulatorConstants.amp_roller, MotorType.kBrushless);
    private final DigitalInput ampSensor = new DigitalInput(0);
    private boolean isBroke;

    public AmpSubsystem() {

    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("intake enco value", AmpEncoder.getPosition());
        isBroke = isBroken();
    }


    public boolean isBroken() {
        return ampSensor.get();
    }

    public void spinAmp(double speed) {
        AmpMotor.set(speed);
    }
    public Command runTilBreak(double speed) {
        return run(()->{
            if (isBroken()){
            spinAmp(speed);
        } else {
            spinAmp(0);
        } 
    });
    }


    @Override
    public void simulationPeriodic() {

    }
}
