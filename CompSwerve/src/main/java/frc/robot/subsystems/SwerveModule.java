// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import static frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase {
  //private final CANSparkMax m_driveMotor;
  //private final CANSparkMax m_turningMotor;

  private final WPI_TalonFX m_fDriveMotor;
  private final WPI_TalonFX m_fTurningMotor;

  //private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  private final TalonFXSensorCollection m_fDriveEncoder;

  public static boolean debug = false;
  private double m_motorEncoderOffset;

  String name;

  int cnt = 0;

  // PID controllers for drive and steer motors
  private final PIDController m_drivePIDController = new PIDController(
      0.1,
      0,
      0);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      1.0,
      3.0,
      0.0,
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeed, kMaxAngularAcceleration));

  //private final PIDController m_turningPIDController = new PIDController(7, 0, 0);

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.05, 0.0);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.05, 0.0);

  private int m_motorChannel;

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
      int turningEncoder,
      double turningEncoderOffset) {
    m_motorChannel = driveMotorChannel;
    //TODO do we need this: m_turnChannel = turningMotorChannel;
    //SmartDashboard.putString("mod" + m_motorChannel, "");
    
    //m_driveMotor =  new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    //m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_fDriveMotor = new WPI_TalonFX(driveMotorChannel);
    m_fTurningMotor = new WPI_TalonFX(turningMotorChannel);

    name = Drivetrain.chnlnames[m_motorChannel - 1];
    SmartDashboard.putString(name, "thing that should work");

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    //m_driveEncoder = m_driveMotor.getEncoder();
    m_fDriveEncoder = m_fDriveMotor.getSensorCollection();

    //m_driveEncoder.setPositionConversionFactor(kDistPerRot); // inches to meters
    //m_driveEncoder.setVelocityConversionFactor(kDistPerRot / 60); // convert RPM to meters per second

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // CANcoder API wants offsets to always be set in degrees but (optionally)
    // angles returned in radians

    m_turningEncoder = new CANCoder(turningEncoder);

    CANCoderConfiguration config = new CANCoderConfiguration();
    // set units of the CANCoder to radians, with velocity being radians per second
    config.sensorCoefficient = 2 * Math.PI / kEncoderResolution; // 4096 for CANcoder
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond; // set timebase to seconds
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180; // should avoid discontinuity at 0 degrees or
                                                                          // 360
    // config.absoluteSensorRange=AbsoluteSensorRange.Unsigned_0_to_360;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.magnetOffsetDegrees = turningEncoderOffset;
    m_turningEncoder.configAllSettings(config);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //TODO check example and see if this is there: 
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

  }

  public void reset(){
    //SmartDashboard.putString("mod" + m_motorChannel, " " + m_turningEncoder.getPosition() + " " + m_turningEncoder.getAbsolutePosition()); 
    //m_driveEncoder.setPosition(0);
    //m_turningEncoder.setPosition(0);
    m_motorEncoderOffset = m_fDriveEncoder.getIntegratedSensorPosition();


    //cnt=0;
  }
  public double heading(){
    return m_turningEncoder.getAbsolutePosition();
  }
  public double cummulativeAngle(){
    return m_turningEncoder.getPosition();
  }
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(cummulativeAngle());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getVelocity(), getRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDistance(), getRotation2d());
  }
  //meters
  public double getDistance(){
     return (getEncoderPosition()/kFalconResolution)*kDistPerRot;
  }

  private double getEncoderPosition(){
    return (m_fDriveEncoder.getIntegratedSensorPosition()-m_motorEncoderOffset);
  }
  //rps
  public double getVelocity() {
      return (m_fDriveEncoder.getIntegratedSensorVelocity()/kFalconResolution)*kDistPerRot;
  }

  public double getRotations() {
    return (getEncoderPosition()/kFalconResolution);
  }

  public void setVelocity(double v) {
    m_fDriveMotor.set(ControlMode.PercentOutput, v);
    //m_fDriveMotor.set(ControlMode.Velocity, v * kFalconResolution / kDistPerRot);
    // System.out.println("???" + kMetersPerSecToTalonVelocity );
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    //SwerveModuleState state = desiredState;  // don't optimize
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation2d());

    double velocity=getVelocity();
    double position=getDistance();
    // Calculate the drive output from the drive PID controller.
    //System.out.printf("Current velocity: %1.2f, Current position: %1.2f, state.speed: %1.2f, state.angle: %1.2f \n", velocity, position, state.speedMetersPerSecond, state.angle.getRadians());
    double driveOutput = m_drivePIDController.calculate(velocity, state.speedMetersPerSecond);
    double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    double turn_angle=getRotation2d().getRadians();

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = -m_turningPIDController.calculate(turn_angle, state.angle.getRadians());
    double turnFeedforward = -m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    double set_drive=driveOutput+driveFeedforward;
    double set_turn=turnOutput+turnFeedforward;



    setVelocity(set_drive);
    m_fTurningMotor.set(ControlMode.PercentOutput, set_turn);
    
    if(debug){
      String s = String.format("POS:%-1.2f Vel %-1.2f vs %-1.2f -> %-1.2f Angle %-3.1f vs %-3.1f -> %-1.2f\n", 
      position, velocity,state.speedMetersPerSecond,set_drive,Math.toDegrees(turn_angle), state.angle.getDegrees(), set_turn); 
      SmartDashboard.putString(name, s);

      // if((cnt%10)==0){
      //     System.out.println(name+" "+s);
      // }
      // cnt++;
    }   
  }

  public void log() {
    String s = String.format("Drive:%-1.2f m Rotations:%-1.2f Angle:%-4.1f Abs:%-4.1f deg, Absdeg:%-4.1f\n", 
    getDistance(), getRotations(), getRotation2d().getDegrees(), Math.toDegrees(cummulativeAngle()), Math.toDegrees(heading()));
    SmartDashboard.putString(name, s);
    //if(name.equals("FL"))
   // System.out.println(s);
  }

  public void setOffset(double offset) {
    m_turningEncoder.configMagnetOffset(offset,20);
  }

  public void setInverted(){
    m_fDriveMotor.setInverted(true);
  }

  public void driveForward(double dist) {
    m_fDriveMotor.set(ControlMode.PercentOutput, dist);
  }
  public void turnAround(double dist) {
    m_fTurningMotor.set(ControlMode.PercentOutput, dist);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
