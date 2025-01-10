package frc.robot.subsystems.manipulators;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

// uses 2 motors
public class ElevatorSubsystem extends SubsystemBase{
// left
    private final CANSparkMax elevatorOne = new CANSparkMax(Constants.ManipulatorConstants.left_Elevator, MotorType.kBrushless);
// right
    private final CANSparkMax elevatorTwo = new CANSparkMax(Constants.ManipulatorConstants.right_Elevator, MotorType.kBrushless);

    public final RelativeEncoder elevatorEncOne = elevatorOne.getEncoder();
    public final RelativeEncoder elevatorEncTwo = elevatorTwo.getEncoder();


    public ElevatorSubsystem() {

    }

    @Override
    public void periodic() {
        // 21
        SmartDashboard.putNumber("elevator enco value 1", elevatorEncOne.getPosition());

        // 22
        SmartDashboard.putNumber("elevator enco value 2", elevatorEncTwo.getPosition());
    }


    public void setElevator(double speed) {
        elevatorOne.set(speed);
        elevatorTwo.set(speed);
    }

    public Command runIndex(double speed) 
    {
        return run(()-> {
            setElevator(speed);
        });

    }

    @Override
    public void simulationPeriodic() {

    }
}
