// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.RestoreAction;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 1.0; // 1 meters per second
  public static final double kMaxAngularSpeed = Math.PI/2; // 1/4 rotation per second
  public static final double kFrontLeftOffset = -13.97;
  public static final double kFrontRightOffset = -286.35;
  public static final double kBackLeftOffset = -222.54;
  public static final double kBackRightOffset = -293.20;

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(3, 7, 9);
  private final SwerveModule m_frontRight = new SwerveModule(4, 8, 12);
  private final SwerveModule m_backLeft = new SwerveModule(2, 6, 10);
  private final SwerveModule m_backRight = new SwerveModule(1, 5, 11);

  static int count = 0;
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(13);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          getGyroAngle(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public Drivetrain() {
    m_gyro.reset();

    m_frontLeft.setOffset(kFrontLeftOffset);
    m_frontRight.setOffset(kFrontRightOffset);
    m_backLeft.setOffset(kBackLeftOffset);
    m_backRight.setOffset(kBackRightOffset);
    resetOdometry();
  }
 public Rotation2d getGyroAngle() {
  return m_gyro.getRotation2d();
 }
  

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroAngle())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    updateOdometry();
    log();
  }

  public void log() {
    SmartDashboard.putNumber("pigeon", m_gyro.getAngle());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(
        getGyroAngle(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    m_poseEstimator.addVisionMeasurement(
        ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
            m_poseEstimator.getEstimatedPosition()),
        Timer.getFPGATimestamp() - 0.3);
  }

  public void resetOdometry(){
    m_gyro.reset();
    m_poseEstimator.resetPosition(getGyroAngle(), 
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    },
    new Pose2d(0, 0, new Rotation2d()));
  }

 

  public void driveForwardAll(double dist) {
    m_backLeft.driveForward(dist);
    m_backRight.driveForward(dist);
    m_frontLeft.driveForward(dist);
    m_frontRight.driveForward(dist);
  }
  public void turnAroundAll(double dist) {
    m_backLeft.turnAround(dist);
    m_backRight.turnAround(dist);
    m_frontLeft.turnAround(dist);
    m_frontRight.turnAround(dist);
  }
 @Override
  public void periodic() {
    log();
    System.out.println(m_gyro.getAngle());
  }
}
