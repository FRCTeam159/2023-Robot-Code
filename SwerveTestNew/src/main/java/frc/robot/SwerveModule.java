// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
  private static final double kWheelRadius = 2;
  private static final int kEncoderResolution = 4096;
  private static final double kDistPerRot = Units.inchesToMeters(kWheelRadius * Math.PI * 2);

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  int  cnt=0;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.2, 0, 0);
  //before bodging:  = new PIDController(.1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      .01,
      0,
      0,
      new TrapezoidProfile.Constraints(
          kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  private int channel;
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel   PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoder
   */
  public SwerveModule(

      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoder) {
      channel = driveMotorChannel;
      SmartDashboard.putString("mod"+channel, "");
    m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(kDistPerRot);
    m_turningEncoder = new CANCoder(turningEncoder);

  
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
    // kEncoderResolution);

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution); -->
    // some code that used to exist

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void reset(){
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
    cnt=0;
  }
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_turningEncoder.getPosition());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), getAngle());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), getAngle());
  }

  public double getDistance(){
    return m_driveEncoder.getPosition();
  }

  public double getVelocity() {
      return m_driveEncoder.getVelocity();
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAngle());

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = 0;//m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getAngle().getRadians(), state.angle.getRadians());

    final double turnFeedforward = 0; //m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);

    String s = String.format("POS:%-1.2f AngleDeg:%-4.1f DriveOut:%-1.2f TurnOut:%-1.2f\n", 
    getDistance(), getAngle().getDegrees(), driveOutput, turnOutput); //Take the values from above and put them into the log function
    cnt++;
    if (channel == 1) {
      
        System.out.println(cnt + " mod " + channel + " " + s);
        if (cnt == 12)
          cnt = 0;
      
    }
    SmartDashboard.putString("mod"+channel, s);
  }

  public void log() {
    String s = String.format("POS:%-1.2f AngleDeg:%-4.1f\n", 
    getDistance(), getAngle().getDegrees()); //Take the values from above and put them into the log function
    // String s = String.format("POS:%-1.2f AngleDeg:%-4.1f DriveOut:%-1.2f TurnOut:%-1.2f\n", 
    // getDistance(), getAngle().getDegrees(), driveOutput, turnOutput); //Take the values from above and put them into the log function
    // //System.out.println("s");
    SmartDashboard.putString("mod"+channel, s);
      
  }

  public void setOffset(double offset) {
    m_turningEncoder.configMagnetOffset(offset,20);
  }

  public void driveForward(double dist) {
    m_driveMotor.set(dist);
  }
  public void turnAround(double dist) {
    m_turningMotor.set(dist);
    // System.out.println(m_turningEncoder.getDeviceID());
    // System.out.println(m_turningEncoder.getAbsolutePosition());
    SmartDashboard.putString("mod" + channel, " " + m_turningEncoder.getPosition() + " " + m_turningEncoder.getAbsolutePosition()); 
  }

}
