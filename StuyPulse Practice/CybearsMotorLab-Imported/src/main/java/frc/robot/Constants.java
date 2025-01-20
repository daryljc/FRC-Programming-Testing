// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Ports {
    public static int LEFT_TOP = 0;
    public static int LEFT_BOTTOM = 1;

    public static int RIGHT_TOP = 2;
    public static int RIGHT_BOTTOM = 3;

    public static int LEFT_A = 4;
    public static int LEFT_B = 5;

    public static int RIGHT_A = 6;
    public static int RIGHT_B = 7;

  }

  public static class Drivetrain {
    public static int DISTANCE_PER_PULSE = 256; //Change
    public static double GEAR_RATIO = 3.0;
    public static double MASS = 50.0;
    public static double WHEEL_RADIUS = 0.75;
    public static double INERTIA = 2.0; // change
    public static double TRACK_WIDTH = 28.0; // change

    public static RobotConfig config;

    public static double autoSpeed = 1.0;
  }

  public static class Field {
    public static double WIDTH = Units.inchesToMeters(323.25);  //  8.21
    public static double LENGTH = Units.inchesToMeters(651.25); // 16.54
  }

}
