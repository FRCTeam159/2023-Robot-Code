// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    static public final double kDriveGearRatio=6.75;  // MK4i drive (standard)
    static public final double kTurnGearRatio=21.429; // MK4i turn (all)
  
    public static final double kWheelRadius = 2;
    public static final int kEncoderResolution = 4096;
    public static final double kDistPerRot = Units.inchesToMeters(kWheelRadius * Math.PI * 2)/kDriveGearRatio;

    public static final double kMaxSpeed = 4; // 4 meters per second
    public static final double kMaxAngularSpeed = 12 * Math.PI; // 3 rotation per second
    public static final double kMaxAngularAcceleration = 6*Math.PI; // 1 rotations/s/s

    public static final double kFrontLeftOffset = -7.3;
    public static final double kFrontRightOffset = 74.5;
    public static final double kBackLeftOffset = 138.2;
    public static final double kBackRightOffset = 64.3;

    public static final double kFrontWheelBase = 23.22; // distance beteen front wheels
	public static final double kSideWheelBase = 23.22;  // distance beteen side wheels

    public static final double kStartArm = 5;

    public static final int kImageWidth = 640;
    public static final int kImageHeight = 480;
}
