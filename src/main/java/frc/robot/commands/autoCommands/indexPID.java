package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.manipulatorCommands.elevatorCommands.elevatorPIDCmdExtend;
import frc.robot.commands.manipulatorCommands.elevatorCommands.elevatorPIDCmdRetract;
import frc.robot.subsystems.manipulators.ElevatorSubsystem;

import java.lang.reflect.Proxy;

// FIXME FIXME URGENT FIXME 
// no pid rn

//error to get attention

public class indexPID extends SequentialCommandGroup {
     public indexPID(ElevatorSubsystem elevatorSubsystem) {
        super(
            new elevatorPIDCmdExtend(elevatorSubsystem, -61),
            new WaitCommand(1.0),
            new elevatorPIDCmdRetract(elevatorSubsystem, -2)
        );
     }
}

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ProxyCommand;
// import frc.robot.subsystems.manipulators.ElevatorSubsystem;

// public class indexPID extends Command {

//     private final ElevatorSubsystem elevatorSubsystem;
//     private final PIDController pidController;
//     private final double point;

//     //FIXME does this work? does it make the command end?
//     private int mode = 0;


//     // FIXME could the low p-value be the reason why the elevator is so slow? or is it mechanical?
//     public indexPID(ElevatorSubsystem elevatorSubsystem, double setpoint) {
//         this.elevatorSubsystem = elevatorSubsystem;
//         point = setpoint;
//         this.pidController = new PIDController(0.048, 0.0, 0.009);
//         pidController.setSetpoint(setpoint);
//         addRequirements(elevatorSubsystem);
//     }

//     @Override
//     public void initialize() {
//         System.out.println("elevator started");
//         pidController.reset();
//     }

//     @Override
//     public void execute() {
//         double speed = pidController.calculate(elevatorSubsystem.elevatorEncOne.getPosition());

//         // FIXME is this sequential?
//         if (elevatorSubsystem.elevatorEncOne.getPosition() <= point ) {
//             elevatorSubsystem.setElevator(speed);
//         } 
//         elevatorSubsystem.setElevator(-2);
//         mode = 1;

//         ProxyCommand
//     }

//     @Override
//     public void end(boolean interrupted) {
//        // elevatorSubsystem.setElevator(0);
//         System.out.println("indexPID ended");
//         mode = 0;
//     }

//     @Override
//     public boolean isFinished() {
//         if (mode == 1)
//         {
//         return true;
//         } else {
//             return false;
//         }
//     }
// }
