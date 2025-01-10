package frc.robot.subsystems.manipulators;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.Encoder;

// uses 2 motors
public class PivotSubsystem extends SubsystemBase{
    // 14
    private final CANSparkMax pivotOne = new CANSparkMax(ManipulatorConstants.left_Pivot, MotorType.kBrushless);
    // 13
    private final CANSparkMax pivotTwo = new CANSparkMax(ManipulatorConstants.right_Pivot, MotorType.kBrushless);
    



  public final RelativeEncoder pivotEncOneBI = pivotOne.getEncoder();
    public final RelativeEncoder pivotEncTwoBI = pivotTwo.getEncoder();


    //DigitalInput input = new DigitalInput(1);
  //  public final DutyCycleEncoder tbEncoder = new DutyCycleEncoder(1);
    

    //public final Encoder pivotEncOne = new Encoder(null, null)
   // public final SparkMaxAlternateEncoder tbEncoder = new SparkMaxAlternateEncoder()
    
    

    public PivotSubsystem() {
         //pivotEncOne.setDistancePerRotation(3.5);
         //pivotEncOne.setPositionOffset(0.57);

         //tbEncoder.setDistancePerRotation(8871);
         // it DOESN go here :(
    }

    @Override
    public void periodic() {

         //double pivVal = pivotEncOne.getDistancePerRotation();
        // left
        SmartDashboard.putNumber("pivot enco value 1 BI", pivotEncOneBI.getPosition());
        // right
        SmartDashboard.putNumber("pivot enco value 2 BI", pivotEncTwoBI.getPosition());
        
       //  SmartDashboard.putNumber("trhough bore", tbEncoder.getAbsolutePosition());
        // amp - 0.945
        // stow - 0.3
        // intake - 0
        // shoot - 0.565


        //SmartDashboard.putNumber("pivot encoder value absolute", (pivotEncOne.getAbsolutePosition() - pivotEncOne.getPositionOffset()));
        // SmartDashboard.putNumber("pivot encoder value get", pivotEncOne.get());
        // SmartDashboard.putNumber("pivot encoder distancepRota", pivotEncOne.getDistancePerRotation());
        // SmartDashboard.putNumber("pivot encoder bal", pivotEncOne.getDistance());
    }


    public void setMotors(double speed) {
        pivotOne.set(speed);
        pivotTwo.set(speed);
    }


    @Override
    public void simulationPeriodic() {

    }
}
