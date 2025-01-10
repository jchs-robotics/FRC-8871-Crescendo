package frc.robot.subsystems.manipulators;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

// uses 2 motors
public class ShooterSubsystem extends SubsystemBase{

    private final CANSparkMax shooterOne = new CANSparkMax(Constants.ManipulatorConstants.left_Shooter, MotorType.kBrushless);
    private final CANSparkMax shooterTwo = new CANSparkMax(Constants.ManipulatorConstants.right_Shooter, MotorType.kBrushless);

    // public final RelativeEncoder shooterEncOne = shooterOne.getEncoder();
    // public final RelativeEncoder shooterEncTwo = shooterTwo.getEncoder();


    public ShooterSubsystem() {

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("shooter enco value 1", shooterEncOne.getPosition());
        // SmartDashboard.putNumber("shooter enco value 2", shooterEncTwo.getPosition());
    }


    public void setShooter(double speed) {
        shooterOne.set(speed);
        shooterTwo.set(-speed);
    }

    public Command manualShooter(DoubleSupplier speed) { 
        return run(()-> {
            setShooter(speed.getAsDouble());
        });
    }
    public Command runShooter(double speed) 
    {
        return new RunCommand(()-> {
            setShooter(speed);
        });

    }






    @Override
    public void simulationPeriodic() {

    }
}
