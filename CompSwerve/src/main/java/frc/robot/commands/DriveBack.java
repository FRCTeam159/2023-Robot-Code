// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class DriveBack extends CommandBase {
  Drivetrain m_drive;
  double target;
  int c = 0;
  /** Creates a new DriveBack. */
  public DriveBack(Drivetrain drive, Integer mode) {
    m_drive = drive;
    if(mode == Autonomous.Center){
      target = -1.98;
    } else if(mode == Autonomous.Outside) {
      target = -Units.inchesToMeters(206);
    }
    
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(-3, 0, 0, false);
    c++;
    if(c%10 == 0){
      System.out.println(m_drive.getPose().getX());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double x = m_drive.getPose().getX();
    return (x-target)<0.05;
  }
}
