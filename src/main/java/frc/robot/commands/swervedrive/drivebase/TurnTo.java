// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.swervedrive.drivebase;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import java.util.logging.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.OdometryThread;

import swervelib.SwerveController;
import swervelib.math.SwerveMath;


public class TurnTo extends Command
{


  public static final double MAX_ERROR_ALLOWED = 10;
  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier  angularVelocitySupplier;
  private final Supplier<Rotation2d> targetAngle;
  private final PIDController snapToAnglePID = new PIDController(1, 0, 0);
  private final boolean onTheFly;


  public TurnTo(
    SwerveSubsystem swerve,
    Rotation2d targetAngle,
    DoubleSupplier vX,
    DoubleSupplier vY) {
        this(swerve, () -> targetAngle, vX, vY);
    }


    public TurnTo(
    SwerveSubsystem swerve,
    Supplier<Rotation2d> targetAngle,
    DoubleSupplier vX,
    DoubleSupplier vY) {
        this(swerve, targetAngle, vX, vY, false, () -> 0.0);
    }


    public TurnTo(
    SwerveSubsystem swerve,
    Supplier<Rotation2d> targetAngle,
    DoubleSupplier vX,
    DoubleSupplier vY,
    Boolean onTheFly,
    DoubleSupplier angularVelocitySupplier) {
        this.swerve = swerve;
        this.targetAngle = targetAngle;
        this.vX = vX;
        this.vY = vY;
        this.onTheFly = onTheFly;
        this.angularVelocitySupplier = angularVelocitySupplier;
        addRequirements(swerve);
        }


        @Override
        public void initialize() {
            snapToAnglePID.reset();
            snapToAnglePID.enableContinuousInput(0, 2* Math.PI);
        }


        @Override
        public void execute() {
            /*double rotationalVelocity = MathUtil.clamp(
                snapToAnglePID.calculate(
                    swerve.getYaw().getRadians(),
                    targetAngle.get().getRadians()),
                    -swerve.getMaxAngularVelocityRadiansPerSec(),
                    swerve.getMaxAngularVelocityRadiansPerSec());*/
            snapToAnglePID.setTolerance(MAX_ERROR_ALLOWED);
            double rotationalVelocity = swerve.getSwerveController().headingCalculate(swerve.getYaw().getRadians(), targetAngle.get().getRadians());
            rotationalVelocity += Math.copySign(0.19, rotationalVelocity);
            
            
            double xVelocity;
            double yVelocity;
            if (!onTheFly) {
                if (vX.getAsDouble() < 0.0) {
                    xVelocity = vX.getAsDouble();
                } else {
                    xVelocity = vX.getAsDouble()
                        * (0.3);
                }
                yVelocity = vY.getAsDouble() * (0.3);
            } else {
                xVelocity = vX.getAsDouble();
                yVelocity = vY.getAsDouble();
            }
           
            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                rotationalVelocity,
                new Rotation2d(swerve
                    .getPose()
                    .getRotation().getRadians())
            ));
            //FIXME post info to smartdashboard
            
            // SmartDashboard.putNumber("SnapAnglePIDOutput", rotationalVelocity);
           // SmartDashboard.putNumber("Drive/TargetAngle", targetAngle.get().getRadians());
           // FIXME ^ uncomment?
            }

            @Override
            public void end(boolean interrupted) {
                swerve.setChassisSpeeds(new ChassisSpeeds());
            }
           
            @Override
            public boolean isFinished() {
                if ((swerve.getPose().getRotation().getDegrees() < targetAngle.get().getDegrees() && swerve.getPose().getRotation().getDegrees() > targetAngle.get().getDegrees() -MAX_ERROR_ALLOWED)
                 || (swerve.getPose().getRotation().getDegrees() > targetAngle.get().getDegrees() && swerve.getPose().getRotation().getDegrees() < targetAngle.get().getDegrees() +MAX_ERROR_ALLOWED)) {
                    return true;
                }
                if (onTheFly && Math.abs(angularVelocitySupplier.getAsDouble()) > 0.1) {
                    return true;
                } else {
                    return false;
                }
            }




}
