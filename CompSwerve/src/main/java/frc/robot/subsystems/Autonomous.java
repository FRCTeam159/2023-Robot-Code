// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.DriveBack;
import frc.robot.commands.DrivePath;


public class Autonomous extends SubsystemBase {
  //PathPlannerTrajectory examplePath = PathPlanner.loadPath("Center", new PathConstraints(4,3));
  Drivetrain m_drive;
  Arm m_arm;
  Claw m_claw;
  SendableChooser<Integer> m_chooser = new SendableChooser<Integer>();
  public static int Center = 0;
  public static int Outside = 1;
  /** Creates a new Autonomous. */
  public Autonomous(Drivetrain drive, Arm arm, Claw claw) {
    m_drive = drive;
    m_arm = arm;
    m_claw = claw;
    m_chooser.addOption("center", 0);
    m_chooser.setDefaultOption("outside", 1);
    SmartDashboard.putData(m_chooser);
    
  }
  public SequentialCommandGroup getCommand(){
    return new SequentialCommandGroup(new AutoPlace(m_arm, m_claw), new DriveBack(m_drive, m_chooser.getSelected()));
    //return new SequentialCommandGroup(new DrivePath(m_drive));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
