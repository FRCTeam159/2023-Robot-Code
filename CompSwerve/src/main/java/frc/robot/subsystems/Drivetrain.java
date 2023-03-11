// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  public static double delx = Units.inchesToMeters(0.5 * kSideWheelBase); // 0.2949 metters
  public static double  dely= Units.inchesToMeters(0.5 * kFrontWheelBase);
  public static double kMaxAcceleration = 1.0;
  public static double kMaxVelocity = 1.0;

  private final Translation2d m_frontLeftLocation = new Translation2d(delx, dely);
  private final Translation2d m_frontRightLocation = new Translation2d(delx, -dely);
  private final Translation2d m_backLeftLocation = new Translation2d(-delx, dely);
  private final Translation2d m_backRightLocation = new Translation2d(-delx, -dely);

  public static String chnlnames[] = { "BL", "BR", "FL", "FR" };

  private final SwerveModule m_frontLeft = new SwerveModule(kFl_Drive, kFl_Turn, kFl_Encoder, kFrontLeftOffset);
  private final SwerveModule m_frontRight = new SwerveModule(kFr_Drive, kFr_Turn, kFr_Encoder, kFrontRightOffset);
  private final SwerveModule m_backLeft = new SwerveModule(kBl_Drive, kBl_Turn, kBl_Encoder, kBackLeftOffset);
  private final SwerveModule m_backRight = new SwerveModule(kBr_Drive, kBr_Turn, kBr_Encoder, kBackRightOffset);

  private final Field2d m_Field2d = new Field2d();

  SwerveModulePosition[] m_positions = {
      new SwerveModulePosition(), new SwerveModulePosition(),
      new SwerveModulePosition(), new SwerveModulePosition() };

  public Drivetrain() {
    SmartDashboard.putData("Field" , m_Field2d);
    m_gyro.reset();

    m_frontLeft.setOffset(kFrontLeftOffset);
    m_frontRight.setOffset(kFrontRightOffset);
    m_backLeft.setOffset(kBackLeftOffset);
    m_backRight.setOffset(kBackRightOffset);
    //m_frontRight.setInverted();
    resetOdometry();
  }

  private void resetPositions() {
    m_frontLeft.reset();
    m_frontRight.reset();
    m_backLeft.reset();
    m_backRight.reset();
    updatePositions();
    
  }

  public void reset() {
    resetOdometry();
    log();
  }

  private void updatePositions() {
    m_positions[0] = m_frontLeft.getPosition();
    m_positions[1] = m_frontRight.getPosition();
    m_positions[2] = m_backLeft.getPosition();
    m_positions[3] = m_backRight.getPosition();
  }

  static int count = 0;
  public final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(13);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      getGyroAngle(),
      m_positions,
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // drive confidence stds
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // turn confidence stds

  public Rotation2d getGyroAngle() {
    return m_gyro.getRotation2d();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    updateOdometry();
    // System.out.println(xSpeed + ""+ ySpeed + "" + rot + "" + fieldRelative + "");
    // log();
  }

  public void log() {
    SmartDashboard.putNumber("pigeon", m_gyro.getAngle());
    m_frontLeft.log();
    m_frontRight.log();
    m_backLeft.log();
    m_backRight.log();

  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    updatePositions();
    m_poseEstimator.update(getGyroAngle(), m_positions);
    m_Field2d.setRobotPose(getPose());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    // m_poseEstimator.addVisionMeasurement(
    // ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
    // m_poseEstimator.getEstimatedPosition()),
    // Timer.getFPGATimestamp() - 0.3);

    
  }

  public void resetOdometry() {
    m_gyro.reset();
    resetPositions();
    m_poseEstimator.resetPosition(getGyroAngle(),
        m_positions,
        new Pose2d(0, 0, new Rotation2d()));
  }
  public Pose2d getPose(){
    return m_poseEstimator.getEstimatedPosition();
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

  //TODO quick fix for autos error remove when auto is programmed
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }


}
