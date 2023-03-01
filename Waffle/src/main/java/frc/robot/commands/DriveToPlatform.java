// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToPlatform extends CommandBase {
  /** Creates a new DriveToPlatform. */
  private Drivetrain m_Drive;

  public DriveToPlatform(Drivetrain drive) {
    m_Drive = drive;
  }

  public void initialize(){

  }

  public void execute(){

  }


  public void driveBack(){
    m_Drive.drive(0, 1, 0, false);
  }
}


