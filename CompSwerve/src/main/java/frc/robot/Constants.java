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
    static public final boolean flagWaffle = false;

    static public final double kDriveGearRatio=6.75;  // MK4i drive (standard)
    static public final double kTurnGearRatio=21.429; // MK4i turn (all)
  
    public static final double kWheelRadius = 2;
    public static final int kEncoderResolution = 4096;
    public static final int kFalconResolution = 2048*10;
    public static final double kDistPerRot = Units.inchesToMeters(kWheelRadius * Math.PI * 2)/kDriveGearRatio;

    public static final double kMaxSpeed = 4; // 4 meters per second
    public static final double kMaxAngularSpeed = 12 * Math.PI; // 3 rotation per second
    public static final double kMaxAngularAcceleration = 6*Math.PI; // 1 rotations/s/s
    //in degrees, need to be set in degrees
    public static final double kFrontLeftOffset = -51.6;
    public static final double kFrontRightOffset = -74.5;
    public static final double kBackLeftOffset = -154.7;
    public static final double kBackRightOffset = 17.2;

    public static final double kFrontWheelBase = flagWaffle? 23.22: 18.625; // distance bewteen front wheels (in)
	public static final double kSideWheelBase = flagWaffle? 23.22: 31;  // distance beteen side wheels (in)

    public static final int kImageWidth = 640;
    public static final int kImageHeight = 480;

    public static final double kBoxTargetArea = 17;
    public static final double kConeTargetArea = 12;
    public static final double kPostTargetArea = 2;

    public static final double kStageOneLength = Units.inchesToMeters(43.18);
    public static final double kStageTwoLength = Units.inchesToMeters(30.59);

    public static final int kFl_Drive = 3;
    public static final int kFl_Turn = 7;
    public static final int kFl_Encoder = 10;

    public static final int kFr_Drive = 4;
    public static final int kFr_Turn = 8;
    public static final int kFr_Encoder = 9;

    public static final int kBr_Drive = 2;
    public static final int kBr_Turn = 6;
    public static final int kBr_Encoder = 12;

    public static final int kBl_Drive = 1;
    public static final int kBl_Turn = 5;
    public static final int kBl_Encoder = 11;

    public static final int kStageOneChannel = 13;
    public static final int kStageTwoChannel = 14;
    public static final int kWristChannel = 15;

    public static final int kClawMotorID1 = 0;
    public static final int kClawMotorID2 = 0;

    // public static final int 

    public static final Object[][] modes = { //1st letter is operation P-pickup, D-Dropoff; 2nd letter is type C-cone, B-box; 3rd letter is location B-bottom, M-middle T-top
        {"PCB", 0, 10, 2}, //modes[x][1] is y-target on camera; modes[x][2] is area-target; modes[x][3] is target type (refer to limelight subsys)
        {"PCM", 0, 10, 2},
        {"PBB", 0, 10, 1},
        {"PBM", 0, 10, 1},
        {"DCB", 0, 10, 3},
        {"DCM", 0, 10, 3},
        {"DCT", 0, 10, 3},
        {"DBB", 0, 10, 0},
        {"DBM", 0, 10, 0},
        {"DBT", 0, 10, 0},
    };
}
