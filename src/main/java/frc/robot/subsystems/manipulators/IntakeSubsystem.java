package frc.robot.subsystems.manipulators;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    // extend intake out
    private final CANSparkMax intakeExtend = new CANSparkMax(Constants.ManipulatorConstants.extendTake, MotorType.kBrushless);


    public final RelativeEncoder intakeEncoder = intakeExtend.getEncoder();
    // spin intake
    private final CANSparkMax spintake = new CANSparkMax(Constants.ManipulatorConstants.spinTake, MotorType.kBrushless);

    public IntakeSubsystem() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake enco value", intakeEncoder.getPosition());
    }



    // FIXME make this two seperate subsystems?
    public void setIntake(double speed) {
        spintake.set(speed);
    }

    public void extendIntake(double speed) {
        intakeExtend.set(speed);
    }

    public void coast()
    {
        spintake.setIdleMode(IdleMode.kCoast);
    }
    
    public void breakit()
    {
        spintake.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void simulationPeriodic() {

    }
}
