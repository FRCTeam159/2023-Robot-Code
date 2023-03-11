// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.*;
import frc.robot.subsystems.Limelight;

/** An example command that uses an example subsystem. */
public class DriveWithGamepad extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final XboxController m_controller;
  private final Drivetrain m_drive;
  //private final Claw m_claw;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithGamepad(Drivetrain drive, XboxController controller) {
    m_drive = drive;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.reset();
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSetter();
    driveWithJoystick(true);
    if (ledSetter()) {
      for (var i = 0; i<m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255,0,0);
      } 
    } else {
      for (var i = 0; i<m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0,255,0);
      }
    }
    //testLimelight();
    //testClaw();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.2)) * kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(Math.pow(MathUtil.applyDeadband(m_controller.getLeftX(), 0.2), 3)) * kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(Math.pow(MathUtil.applyDeadband(m_controller.getRightX(), 0.2), 5))* kMaxAngularSpeed;

    m_drive.drive(xSpeed + 0.001, ySpeed, rot, fieldRelative);
    // m_drive.driveForwardAll(xSpeed);
    // m_drive.turnAroundAll(rot);
  }

  public boolean ledSetter()
  {

    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.2)) * kMaxSpeed;
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.2)) * kMaxSpeed;

    if(xSpeed > 0 && ySpeed > 0)
    {
      return true;
    } else {
      return false;
    }
  
  }
}
