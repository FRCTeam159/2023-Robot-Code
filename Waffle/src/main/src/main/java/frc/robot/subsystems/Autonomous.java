// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DrivePath;


public class Autonomous extends SubsystemBase {
  PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example path", new PathConstraints(4,3));
  Drivetrain m_drive;
  /** Creates a new Autonomous. */
  public Autonomous(Drivetrain drive) {
    m_drive = drive;
    
  }
  public SequentialCommandGroup getCommand(){
      
    return new SequentialCommandGroup(new DrivePath(m_drive));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
