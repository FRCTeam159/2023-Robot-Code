// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.BNO055;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort;


import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {
  public static double dely = Units.inchesToMeters(0.5 * kSideWheelBase); // 0.2949 metters
  public static double delx = Units.inchesToMeters(0.5 * kFrontWheelBase);
  public static double kMaxAcceleration = 1.0;
  public static double kMaxVelocity = 1.0;

  private final Translation2d m_frontLeftLocation = new Translation2d(-delx, -dely);
  private final Translation2d m_frontRightLocation = new Translation2d(-delx, dely);
  private final Translation2d m_backLeftLocation = new Translation2d(delx, -dely);
  private final Translation2d m_backRightLocation = new Translation2d(delx, dely);

  public static String chnlnames[] = { "BR", "BL", "FL", "FR" };

  private final SwerveModule m_frontLeft = new SwerveModule(kFl_Drive, kFl_Turn,1);
  private final SwerveModule m_frontRight = new SwerveModule(kFr_Drive, kFr_Turn,2);
  private final SwerveModule m_backRight = new SwerveModule(kBr_Drive, kBr_Turn,3);
  private final SwerveModule m_backLeft = new SwerveModule(kBl_Drive, kBl_Turn, 4);

  private final Field2d m_Field2d = new Field2d();

  static int count = 0;
  //private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(13);
  // private AHRS m_gyro = new AHRS(I2C.Port.kOnboard);
  //private AHRS m_gyro = new AHRS();
  ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  AHRS m_navx  = new AHRS(); // kUSB1 - outside type 
  private int[] bnoOffsets = {0, -42, -8, -24, -3, 0, 2, 299, -59, -25, 523};
  BNO055 m_BNOs;


  SwerveModulePosition[] m_positions = {
      new SwerveModulePosition(), new SwerveModulePosition(),
      new SwerveModulePosition(), new SwerveModulePosition() };

  public Drivetrain() {
    SmartDashboard.putData("Field" , m_Field2d);
    m_BNOs = BNO055.getInstance(
      BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
      BNO055.vector_type_t.VECTOR_EULER,
      I2C.Port.kMXP,
      BNO055.BNO055_ADDRESS_A,
      bnoOffsets
      );
    m_BNOs.reset();
    m_navx.reset();
    m_gyro.reset();

    m_frontLeft.setDriveInverted();
    m_backLeft.setDriveInverted();

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
    if (m_gyro != null) {
      return m_gyro.getRotation2d().unaryMinus();
    } else {
      return new Rotation2d();
    }
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
    log();
  }

  public void log() {
    SmartDashboard.putNumber("BNO055 heading: ", m_BNOs.getRotation2d().getDegrees());
    SmartDashboard.putNumber("NAVX heading: ", m_navx.getRotation2d().getDegrees());
    SmartDashboard.putNumber("ADXR heading: ", m_gyro.getRotation2d().getDegrees());
    // m_frontLeft.log();
    // m_frontRight.log();
    // m_backLeft.log();
    // m_backRight.log();
    
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public Command exampleMethodCommand() {
	return null;
}

}
