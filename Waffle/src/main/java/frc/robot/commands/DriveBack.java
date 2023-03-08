// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveBack extends CommandBase {
  private PIDController m_xController = new PIDController(0.1, 0, 1);
  Drivetrain m_drive;
  double m_target;
  /** Creates a new DriveBack. 
   * @param i
   * @param m_drive */
  public DriveBack(Drivetrain drive, double i) {
    m_drive = drive;
    m_target = i;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xController.setTolerance(0.05, .08);
    m_xController.setSetpoint(m_target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = m_drive.getPose();
    double x = pose.getX();
    double error = m_xController.calculate(x, m_target);
    m_drive.drive(error, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_xController.atSetpoint();
  }
}
