// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveBack;

public class Autonomous extends SubsystemBase {
  //PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example path", new PathConstraints(4,3));
  public static boolean autoReset = false;
  Drivetrain m_drive;
  
  /** Creates a new Autonomous. */
  public Autonomous(Drivetrain drive) {
    m_drive = drive;
    //SmartDashboard.putBoolean("autoReset", autoReset);
  }
  public SequentialCommandGroup getCommand(){   
    //m_timer.reset();
   return new SequentialCommandGroup(new DriveBack(m_drive,-2));
     //return new SequentialCommandGroup(new DrivePath(m_drive));
  }
  @Override
  public void periodic() {
  }
}
