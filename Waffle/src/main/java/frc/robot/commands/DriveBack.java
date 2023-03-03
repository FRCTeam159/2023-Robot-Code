// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;


public class DriveBack extends CommandBase {
  //private final RamseteController m_ramsete = new RamseteController();
  private final PIDController m_controller= 
    new PIDController(0.5, 0, 0);
  Drivetrain m_drive;
  double m_distance;
  Trajectory m_trajectory;
  Timer m_timer = new Timer();
  double elapse;
  double runtime;
  /** Creates a new AutoDouble. */
  public DriveBack(Drivetrain drive, double distance) {
    m_drive = drive;
    m_distance = distance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
    m_trajectory = getStraightPath(m_distance);
    System.out.println(m_trajectory);
    m_drive.resetOdometry(m_trajectory.getInitialPose());
    runtime = m_trajectory.getTotalTimeSeconds();
    */
    m_timer.start();
    m_timer.reset();
    elapse = 0;
    m_controller.setTolerance(0.07, 0.01);

  }


  public static Trajectory getStraightPath(double distance) {
    List<Pose2d> points = new ArrayList<>();
    points.add(new Pose2d(distance, 0, new Rotation2d(0.0)));
    points.add(new Pose2d(0, 0, new Rotation2d(0.0)));
    //Collections.reverse(points);
    TrajectoryConfig config = new TrajectoryConfig(Drivetrain.kMaxVelocity, Drivetrain.kMaxAcceleration);
    config.setReversed(true);
    Trajectory Traj = TrajectoryGenerator.generateTrajectory(points, config); 
    return Traj;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    /*
    
    if (elapse<0){
      return;
    }
    Trajectory.State reference = m_trajectory.sample(elapse);
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    System.out.println(m_drive.getPose());
    //System.out.println("time: " + elapse + "speeds:" + speeds);
    m_drive.odometryDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    */
    elapse = m_timer.get();
    Pose2d pose = m_drive.getPose();
    double correction = -kMaxSpeed*m_controller.calculate(pose.getX(), m_distance);
    System.out.format("correction: %-3.1f x: %-3.1f angle: %-3.1f \n", correction, pose.getX(), pose.getRotation().getDegrees());
    m_drive.drive(correction, 0, 0, true);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Autonomous.totalRuntime += elapse;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(true) {
      System.out.println("drove backsies");
    }
    return m_controller.atSetpoint();
    
  }
}
