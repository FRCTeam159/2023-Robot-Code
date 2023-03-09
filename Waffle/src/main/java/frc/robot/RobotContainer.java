// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveBack;
import frc.robot.commands.DrivePath;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.DriveToPlatform;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.PoseArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
//import frc.robot.subsystems.Camera;
//import frc.robot.subsystems.DetectorAprilTag;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.TargetMgr;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Autonomous;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_Controller = new XboxController(0);
  //Subsystems
  private final Drivetrain m_Drivetrain = new Drivetrain();
  private final Autonomous m_auto = new Autonomous(m_Drivetrain);
  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_Drivetrain, m_Controller);

  //private final Camera m_Camera = new Camera();
  private final TargetMgr m_TargetMgr = new TargetMgr();
  public final Limelight m_Limelight = new Limelight();
  private final Arm m_Arm = new Arm(m_Limelight);

  //commands
   
  //private final DetectorAprilTag m_apriltag = new DetectorAprilTag(m_Camera);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Drivetrain.setDefaultCommand(m_DriveWithGamepad);
    // Configure the button bindings
    configureBindings();
  }
  public void robotInit() {
   // m_apriltag.start();
   m_Limelight.start();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {}

  public void autonomousInit() {
    m_Drivetrain.reset();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_auto.getCommand();
  }
}
