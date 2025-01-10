// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TurnTo;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import javax.naming.PartialResultException;

import frc.robot.commands.autoCommands.indexPID;
import frc.robot.commands.autoCommands.aimPivot;
import frc.robot.commands.autoCommands.autoIntake;
import frc.robot.commands.autoCommands.shooting;
import frc.robot.commands.lights.blinky;
import frc.robot.commands.lights.defaultLights;
import frc.robot.commands.lights.offLights;
import frc.robot.commands.lights.testHalfLights;
import frc.robot.commands.lights.varyingLights;
import frc.robot.commands.manipulatorCommands.elevatorCommands.elevatorPIDCmdExtend;
import frc.robot.commands.manipulatorCommands.manShooter;
import frc.robot.commands.manipulatorCommands.shooterCmd;
import frc.robot.commands.manipulatorCommands.ampCommands.ampCmd;
import frc.robot.commands.manipulatorCommands.elevatorCommands.elevatorManCmd;
import frc.robot.commands.manipulatorCommands.intakeCommands.intakeCmdExtend;
import frc.robot.commands.manipulatorCommands.intakeCommands.spinIntakeCmd;
import frc.robot.commands.manipulatorCommands.pivotCommands.rotatePIDCmdUp;
import frc.robot.commands.manipulatorCommands.pivotCommands.rotatePivotCmd;
import frc.robot.commands.manipulatorCommands.pivotCommands.zeroPivot;
import frc.robot.subsystems.manipulators.AmpSubsystem;
import frc.robot.subsystems.manipulators.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.IntakeSubsystem;
import frc.robot.subsystems.manipulators.PivotSubsystem;
import frc.robot.subsystems.manipulators.ShooterSubsystem;

import frc.robot.commands.manipulatorCommands.elevatorCommands.elevatorPIDCmdRetract;
import frc.robot.commands.manipulatorCommands.pivotCommands.rotatePIDCmdDown;
import frc.robot.commands.manipulatorCommands.intakeCommands.intakeCmdRetract;
import frc.robot.commands.manipulatorCommands.intakeCommands.intakeManCmd;
import frc.robot.subsystems.sensors.Lighting;
import frc.robot.subsystems.sensors.Limelight;

import com.fasterxml.jackson.core.json.ReaderBasedJsonParser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  private final SendableChooser<Command> autoChooser;
  private final Lighting leds = new Lighting();
  private final Limelight limeLight = new Limelight();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final AmpSubsystem ampSubsystem = new AmpSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  //PS5Controller driverPS5 = new PS5Controller(0);
  XboxController driverPS5 = new XboxController(0);

   XboxController manipulatorController = new XboxController(1);
   GenericHID logicontroller = new GenericHID(4);
   GenericHID shoeBoxOne = new GenericHID(2);
   GenericHID shoeBoxTwo = new GenericHID(3);

  //GenericHID manipulatorController = new GenericHID(1);

  //PS5Controller jen = new PS5Controller(2);

  CommandXboxController commandManipulatorController = new CommandXboxController(1);
  Trigger rightTrig = commandManipulatorController.rightTrigger();
  Trigger leftTrig = commandManipulatorController.leftTrigger();
  
  //PS5Controller manipulatorController = new PS5Controller(1);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  { 
    // adds auto cmds to pathplanner
    NamedCommands.registerCommand("pivotSpeaker", new rotatePIDCmdUp(pivotSubsystem, -26.4));
    NamedCommands.registerCommand("pivotIntake", new rotatePIDCmdDown(pivotSubsystem, 15.3));
    
    NamedCommands.registerCommand("shoot", shooterSubsystem.runShooter(1));
    NamedCommands.registerCommand("shooting", new shooterCmd(shooterSubsystem, 1));
    NamedCommands.registerCommand("stopShoot", shooterSubsystem.runShooter(0));
    
    NamedCommands.registerCommand("index", new indexPID(elevatorSubsystem));
    
    NamedCommands.registerCommand("extendIntake", new intakeCmdExtend(intakeSubsystem, 35.5));
    // NamedCommands.registerCommand("intakeNote", Commands.parallel(new spinIntakeCmd(intakeSubsystem, 0.45), new shooterCmd(shooterSubsystem, -1)));


   // NamedCommands.registerCommand("index", indexCmd());
    NamedCommands.registerCommand("pivotToSpeaker", new aimPivot(pivotSubsystem));
    NamedCommands.registerCommand("intakeSeq", intakeSequence());
    NamedCommands.registerCommand("aimShoot", aimShooterSequence());
    NamedCommands.registerCommand("autoIntake", new autoIntake(intakeSubsystem, shooterSubsystem, pivotSubsystem));



    NamedCommands.registerCommand("stow", zeroSubsystems());

    // Configure the trigger bindings
    drivebase.zeroGyro();
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverPS5.getRightX(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverPS5.getRightY(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverPS5.getLeftX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverPS5::getAButton,//driverPS5::getTriangleButtonPressed,
                                                                   driverPS5::getBButton,//driverPS5::getCrossButtonPressed,
                                                                   driverPS5::getYButton,//driverPS5::getSquareButtonPressed,
                                                                   driverPS5::getXButton);//driverPS5::getCircleButtonPressed);

                                                                   
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND),
        () -> driverPS5.getLeftX(),
        () -> driverPS5.getLeftY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(driverPS5.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND),
    //     () -> -MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND));

// FIXME

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -0.8 * MathUtil.applyDeadband(driverPS5.getLeftY(), OperatorConstants.RIGHT_Y_DEADBAND),
        () -> 0.8 * MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.LEFT_X_DEADBAND));
      
    Command driveFieldOrientedAnglularVelocityxbox = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverPS5.getRawAxis(1), OperatorConstants.RIGHT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverPS5.getRawAxis(1), OperatorConstants.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getRawAxis(4), OperatorConstants.LEFT_X_DEADBAND));

    Command driveFieldOrientedAnglularVelocityJen = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverPS5.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND))
        ;
    Command driveFieldOrientedPadSim = drivebase.simDriveCommandPad(
        () -> MathUtil.applyDeadband(driverPS5.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> -driverPS5.getPOV(0));

    Command driveFieldOrientedPadAngle = drivebase.driveCommandAngle(
        () -> MathUtil.applyDeadband(driverPS5.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> -driverPS5.getPOV(0));
    


    Command driveFieldOrientedAnglularVelocityInv = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverPS5.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND));

    Command driveFieldOrientedPadSimInv = drivebase.simDriveCommandPad(
        () -> -MathUtil.applyDeadband(driverPS5.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> -driverPS5.getPOV(0));

    Command driveFieldOrientedPadAngleInv = drivebase.driveCommandAngle(
        () -> -MathUtil.applyDeadband(driverPS5.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> -driverPS5.getPOV(0));

    Command shoot = shooterSubsystem.manualShooter(()-> -manipulatorController.getRawAxis(5));
    shooterSubsystem.setDefaultCommand(shoot);
    

    var alliance = DriverStation.getAlliance();
    boolean isred = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;

    if (isred){
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedPadSimInv);
    leds.setDefaultCommand(leds.red());
  } else {
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedPadSim);
    leds.setDefaultCommand(leds.blue());
  }
  
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    new JoystickButton(driverPS5,6).whileTrue(drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverPS5.getRightY() * 0.4, OperatorConstants.RIGHT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getRightX() * 0.4, OperatorConstants.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS5.getLeftX() * 0.25, OperatorConstants.LEFT_X_DEADBAND)));

    // new JoystickButton(driverPS5, 11).whileTrue(new elevatorManCmd(elevatorSubsystem, -0.5));
    new Trigger(()-> driverPS5.getPOV() == 0)
                .onTrue(new TurnTo(
                  drivebase,
                  ()-> Rotation2d.fromDegrees(90),
                  ()-> -driverPS5.getRightY(),
                  ()-> -driverPS5.getRightX(),
                  true,
                  () -> MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)));

  

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //FIXME delete?
    /*new varyingLights(leds,
    () -> driverController.getRawAxis(3),
    () -> driverController.getRawAxis(4));*/
// FIXME uncomment below line
   // new Trigger(()-> driverPS5.getL2Button()).whileTrue(new ProxyCommand(HangSequence())).onFalse(new ProxyCommand(retractSequence()));
    new JoystickButton(driverPS5, 9).onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverPS5, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverPS5,
                       2).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
       new Pose2d(new Translation2d(1.3, 5.5), Rotation2d.fromDegrees(180)))
                              ));
    new JoystickButton(driverPS5,
                       4).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
       new Pose2d(new Translation2d(15, 1), Rotation2d.fromDegrees(130)))
                              ));
    new JoystickButton(driverPS5,
                       1).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
       new Pose2d(new Translation2d(1.8, 7.5), Rotation2d.fromDegrees(270)))
                              ));
    //new JoystickButton(driverPS5, 6).whileTrue(new elevatorPIDCmd(elevatorSubsystem, -0.1));
    //new JoystickButton(driverPS5, 4).onTrue(new testHalfLights(leds));
    //new JoystickButton(driverPS5, 5).whileTrue(new elevatorPIDCmd(elevatorSubsystem, 0.1));
    new JoystickButton(driverPS5, 8).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    //new JoystickButton(driverPS5, 2).whileTrue(new ProxyCommand(new intakeCmd(intakeSubsystem, 20.69)));
    //new JoystickButton(driverPS5, 3).whileTrue(new spinIntakeCmd(intakeSubsystem, 1));
    //new JoystickButton(driverPS5, 1).whileTrue(new ProxyCommand(new intakeCmd(intakeSubsystem, -16)));


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Manipulators ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ // // 


    // ~~~ Encoder positions

// TODO pivot DOWN soft limit

    // pivot - aim speaker: -24.7 to -31.8  
    // pivot - amp: -47.4
    // pivot - intake: 15.3
    // pivot - stow (0): 0 

// TODO pivot source intake: -16.6 (somwhere aroudn this)


    // elevator - index: -61.02
    // elevator - amp: -257
    // elevator - stow(0): -2

    // intake - extend: ~35.5 (hopefully hits hardstop and doesnt skip gears (skipping messes up stow))
    // intake - stow(0): 8 (prevents intake from breaking)


// pov button values
    // up - 0
    // right - 90
    // left - 270
    // down - 180
    
// FIXME pivot spark on coast
    // ~~~~~~~~~~ Dpad ~~~~~~~~~~ //
    // down - stow
    new POVButton(manipulatorController, 180).onTrue(zeroSubsystems());

    // left - ground intake
    //new POVButton(manipulatorController, 270).onTrue(intakeSequence());
     new POVButton(manipulatorController, 270).onTrue(intakeSequence());
    
     // right - amp position
    new POVButton(manipulatorController, 90).onTrue(ampIntakeSequence());
    // up - aim pivot (shooter)
    new POVButton(manipulatorController, 0).onTrue(aimShooterSequence());


    // shoebox set positions
    new JoystickButton(shoeBoxTwo, 1).onTrue(zeroSubsystems());
    new JoystickButton(shoeBoxTwo, 2).onTrue(intakeSequence());
    new JoystickButton(shoeBoxTwo, 3).onTrue(aimShooterSequence());

    new JoystickButton(shoeBoxTwo, 4).onTrue(Commands.parallel(new rotatePIDCmdUp(pivotSubsystem, -21), new shooterCmd(shooterSubsystem, -1)));
    // -21
    
    
    
    // shoebox other commands
    
    // new JoystickButton(shoeBoxOne, 1).whileTrue(new rotatePivotCmd(pivotSubsystem, 0.75));
    // new JoystickButton(shoeBoxOne, 2).whileTrue(new elevatorManCmd(elevatorSubsystem, -1));
    new JoystickButton(shoeBoxOne, 1).onTrue(ampIntakeSequence());
    new JoystickButton(shoeBoxOne, 2).onTrue(aimAmpSequence());
    new JoystickButton(shoeBoxOne, 3).whileTrue(new ampCmd(ampSubsystem, 1, leds));
    new JoystickButton(shoeBoxOne, 4).whileTrue(new ampCmd(ampSubsystem, -1, leds));
    new JoystickButton(shoeBoxOne, 5).onTrue(indexCmd());


    // square - 1
    // x - 2
    // circle - 3
    //triangle - 4

    // left joy button - 11
    // right joy button - 12

    
    // ~~~~~~~~~~ letters ~~~~~~~~~~ //
    //  switch to index// y/triangle - index  4 | logi - 4
    new JoystickButton(manipulatorController, 4).onTrue(indexCmd());
    
    // source intake or amp score
    // ps - . | xbox - . | logi - 2
   // new JoystickButton(manipulatorController, 2).whileTrue(new intakeManCmd(intakeSubsystem, 0.65)).whileFalse(new intakeCmdRetract(intakeSubsystem, 0));
   //new JoystickButton(manipulatorController, 2).onTrue(new rotatePIDCmdUp(pivotSubsystem, -16.738));
   new JoystickButton(manipulatorController, 2).onTrue(aimAmpSequence());

    // man elevator
    // square - manual elevator extend (ps 2/ xbox 1) | logi - 1
    new JoystickButton(manipulatorController, 1).whileTrue(new elevatorManCmd(elevatorSubsystem, 1));
    
    // retract (ps 2/ xbox 3) | logi - 3
    new JoystickButton(manipulatorController, 3).whileTrue(new elevatorManCmd(elevatorSubsystem, -1));

// ~~~~~~~~~~ Joysticks ~~~~~~~~~~ //
   // joy buttons
// man intake + shooter (rev) (ps 11/ xbox 9) | logi - 9
    //new JoystickButton(manipulatorController, 11).whileTrue(new spinIntakeCmd(intakeSubsystem, 0.65)).and(new shooterCmd(shooterSubsystem, -1));
    
    new JoystickButton(manipulatorController, 9).whileTrue(Commands.parallel(new spinIntakeCmd(intakeSubsystem, 0.65), new shooterCmd(shooterSubsystem, -1)));

   // new JoystickButton(manipulatorController, 10).whileTrue(Commands.parallel(new spinIntakeCmd(intakeSubsystem, -1), new shooterCmd(shooterSubsystem, 1)));

    // man spin shooter (ps 12/ xbox 10) | logi - 10
    new JoystickButton(manipulatorController, 10).whileTrue(new shooterCmd(shooterSubsystem, 1));
    

// joy axis
// reverse intake?
    // if (manipulatorController.getRawAxis(1) > 0.1) {
    //   new shooterCmd(shooterSubsystem, -1);
    // }


   // new JoystickButton(manipulatorController, 3).whileTrue(new shooterCmd(shooterSubsystem, 1));
    //new JoystickButton(manipulatorController, 8).whileTrue(new spinIntakeCmd(intakeSubsystem, 1));
    //new JoystickButton(manipulatorController, 9).whileTrue(new spinIntakeCmd(intakeSubsystem, -1));

    // new JoystickButton(manipulatorController, 11).whileTrue(new intakeCmd(intakeSubsystem, 36.5710));
    // new JoystickButton(manipulatorController, 12).whileTrue(new intakeCmd(intakeSubsystem, 0));
   
    // ~~~~~~~~~~ bumper ~~~~~~~~~~ //
    // right bumper - amp intake 6 | logi - 6 | xbox - 6
    //new JoystickButton(manipulatorController, 6).whileTrue(ampSubsystem.runTilBreak(-0.5));
      new JoystickButton(manipulatorController, 6).whileTrue(new ampCmd(ampSubsystem, -1, leds));


   // new JoystickButton(manipulatorController, 6).whileTrue(new ampCmd(ampSubsystem, -0.5, leds));
    // temp fix bc broken controller
    //new JoystickButton(manipulatorController, 12).whileTrue(new ampCmd(ampSubsystem, -0.5, leds));


    // left bumper - amp outtake 5 | logi - 5 | xbox - 5
    new JoystickButton(manipulatorController, 5).whileTrue(new ampCmd(ampSubsystem, 1, leds));
   

    // ~~~~~~~~~~ Triggers ~~~~~~~~~~ // 
    // right trig - button 8/axis 3
    // possible manPivot up
   
    rightTrig.whileTrue(new rotatePivotCmd(pivotSubsystem, -0.75));
    //new JoystickButton(manipulatorController, 8).whileTrue(new rotatePivotCmd(pivotSubsystem, -0.25));
    

    // left trig - button 7/axis 2
    // possible manPivot down
    // FIXME ps - 7, xbox 8
    
    leftTrig.whileTrue(new rotatePivotCmd(pivotSubsystem, 0.75));
    //new JoystickButton(manipulatorController, 7).whileTrue(new rotatePivotCmd(pivotSubsystem, 0.25));

    //new Trigger

    
    // man intake in (logi - 7)
    //new JoystickButton(manipulatorController, 7).onTrue(new intakeCmdRetract(intakeSubsystem, 39));
    new JoystickButton(manipulatorController, 7).whileTrue(new intakeManCmd(intakeSubsystem, 1));
   // man intake out (logi - 8)
    //new JoystickButton(manipulatorController, 8).onTrue(new intakeCmdExtend(intakeSubsystem, 8));
    new JoystickButton(manipulatorController, 8).whileTrue(new intakeManCmd(intakeSubsystem, -1));
    
    
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
  
  public void setDriveMode()
  {
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  /*public void lightStart()
  {
    new offLights(leds);
  }*/
  

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Manipulator Commands ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  
  // FIXME not retracting enough
  public Command indexCmd() {
     return new SequentialCommandGroup(
      new elevatorPIDCmdExtend(elevatorSubsystem, -61.02),
      //new elevatorPIDCmd(elevatorSubsystem, 0)
      new elevatorPIDCmdRetract(elevatorSubsystem, -2)
      );
  }

  // FIXME not working
  public Command intakeSequence() {
    return new SequentialCommandGroup(
      // extend intake
      // hard hard limit is 38
      new intakeCmdExtend(intakeSubsystem, 37)

      // lower pivot
       ,new ParallelCommandGroup( new rotatePIDCmdDown(pivotSubsystem, 16.25),

        // spin shooter (reverse) + spin intake
            new shooterCmd(shooterSubsystem, -1),
            new spinIntakeCmd(intakeSubsystem, 0.65) 
      )
    );
  }

  // TODO fine tune height
  // rotate pivot and extend elevator to grab note
  public Command ampIntakeSequence() {
    // return new SequentialCommandGroup(
    // // set pivot to a position where elevator can extend
    // new rotatePIDCmdUp(pivotSubsystem, -40), // -42.4 --> -40 --> 41? perchance (not tested)
    // new elevatorPIDCmdExtend(elevatorSubsystem, -220) // 232 (bottom) --> 220 --> 215 --> 207 --> 211
    
    
    
    // extend elevator AND pivot AND spin amp
    return new ParallelCommandGroup(
        new rotatePIDCmdUp(pivotSubsystem, -41.5),
        new elevatorPIDCmdExtend(elevatorSubsystem, -210)

        //, new ampCmd(ampSubsystem, -1, leds)

        


        );   
      }
    
    // amp score command
      public Command aimAmpSequence() {
        return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new rotatePIDCmdUp(pivotSubsystem, -44),
            new elevatorPIDCmdExtend(elevatorSubsystem, -190) // -150 --> -215 --> -200 --> -180 --> 190
        ),
        new elevatorPIDCmdRetract(elevatorSubsystem, -150) // -177 --> -165 --> -150
        );
      }
      
      




      public Command HangSequence() {
        return new ParallelCommandGroup(
          leds.orangePulse(),
          new ParallelCommandGroup(
    // set pivot to a position where elevator can extend
    new rotatePIDCmdUp(pivotSubsystem, -42.4),
    new SequentialCommandGroup(new WaitCommand(0.05), 
    new elevatorPIDCmdExtend(elevatorSubsystem, -200))
    
    
    // extend elevator AND pivot AND spin amp
      //new ParallelCommandGroup(
        //new rotatePIDCmd(pivotSubsystem, -65),
        //new ampCmd(ampSubsystem, 1),
        //new elevatorPIDCmd(elevatorSubsystem, -254)
      //)
    )
    );   
  }
    public Command retractSequence() {
    return new ParallelCommandGroup(
    // set pivot to a position where elevator can extend
    new elevatorPIDCmdRetract(elevatorSubsystem, -8),
    new SequentialCommandGroup(new WaitCommand(0.05),
    new rotatePIDCmdDown(pivotSubsystem, 0))
    
    
    // extend elevator AND pivot AND spin amp
      //new ParallelCommandGroup(
        //new rotatePIDCmd(pivotSubsystem, -65),
        //new ampCmd(ampSubsystem, 1),
        //new elevatorPIDCmd(elevatorSubsystem, -254)
      //)
    );   
  }
  // TODO angle
  // aim shooter
  public Command aimShooterSequence() {
    return new ParallelCommandGroup(
        new shooterCmd(shooterSubsystem, 1),
        // built in encoder
        
        
        // left: -19.45 | right: -19.52 # naur :(
        // 23?

        new rotatePIDCmdUp(pivotSubsystem, -23.25) //TODO

        // through bore
    //  new rotatePIDCmdUp(pivotSubsystem, 0.565)

    );
  }




  // resets and zeroes all motors (stow position cmd)
  public Command zeroSubsystems() {
    return new SequentialCommandGroup(
    /* Order of zero-ing
     * Motors (amp, intake, shooter) off
     * Elevator down (0)
     * Pivot down (0)
     * Bring in intake(0)
     */

    // TODO
    // test zero subsys cmd
    // PID testing
    // - we want to increase speed A LOT

    new ParallelCommandGroup(
      new shooterCmd(shooterSubsystem, 0),
      new spinIntakeCmd(intakeSubsystem, 0),
      new ampCmd(ampSubsystem, 0, leds)
    ),

    // new elevatorPIDCmdRetract(elevatorSubsystem, -2),
    
    // new zeroPivot(pivotSubsystem, 0),
    

    new ParallelCommandGroup(
      new elevatorPIDCmdRetract(elevatorSubsystem, -2),
      new zeroPivot(pivotSubsystem, 0)
    ),

    new intakeCmdRetract(intakeSubsystem, 8)   
    );
  }

  

}