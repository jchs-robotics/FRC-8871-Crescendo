// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (100) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  
  public static class SensorConstants {
    public static final int ledLength = 28;
    public static final int ledPort = 0;
    //public static final int ampSensor = 0;

  }
  
  public static final class Auton
  {

    public static final PIDConstants TranslationPID = new PIDConstants(1.3, 0.13, 0.01);
    public static final PIDConstants angleAutoPID   = new PIDConstants(1.5, 0.08, 0.05);

    public static final double MAX_ACCELERATION = 3.5;
  }
  //test sensor
  //zero the pivot (up until black line)

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ManipulatorConstants
  {
    //FIXME change values accordingly
    public static final int spinTake = 19;
    public static final int extendTake = 18;

    public static final int left_Elevator = 21;
    public static final int right_Elevator = 22;

    public static final int right_Shooter = 16;
    public static final int left_Shooter = 17;

    public static final int right_Pivot = 13;
    public static final int left_Pivot = 14;

    //public static final double pivot_Conversion;

    public static final int amp_roller = 20;

    //public static final double amp_Conversion;

  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.005;
    public static final double RIGHT_Y_DEADBAND = 0.005;
    public static final double TURN_CONSTANT    = 6;
  }
}
