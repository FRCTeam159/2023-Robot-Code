// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class DriveToTarget extends CommandBase {
  /** Creates a new DriveToTarget. */
  PIDController turnController = new PIDController(1.0,0,0);
  PIDController areaController = new PIDController(1.0,0,0);

  Drivetrain m_drive;
  public DriveToTarget(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    turnController.setTolerance(5, 5);
    areaController.setTolerance(5, 2);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.setSetpoint(0);
    areaController.setSetpoint(20);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double offset = Limelight.tx;
    double area = Limelight.ta;
    double offsetCorrection = turnController.calculate(offset);
    double areaCorrection = areaController.calculate(area);

    m_drive.drive(-areaCorrection, 0, -offsetCorrection, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("have target");
    return ((turnController.atSetpoint() && areaController.atSetpoint()) || !Limelight.haveTarget); 
  }
}
