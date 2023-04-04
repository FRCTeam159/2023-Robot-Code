// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivePath extends CommandBase {

  private Drivetrain m_drive;
  private Trajectory m_trajectory;
  private final Timer m_timer = new Timer();
  double runtime;
  double elapsed = 0;
  int states;
  int intervals;

  double maxV;
  double maxA;
  double last_time;

  private final PPHolonomicDriveController m_ppcontroller = new PPHolonomicDriveController(
    new PIDController(6,0,0), new PIDController(2,0,0), new PIDController(2,0,0));

  public DrivePath(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drive.reset();
    System.out.println("DRIVEPATH_INIT");
    
    maxV=Drivetrain.kMaxVelocity;
    maxA=Drivetrain.kMaxAcceleration;

    m_trajectory=getTrajectory();
    if(m_trajectory ==null){
      System.out.println("failed to create Trajectory");
      return;
    }

    
    runtime = m_trajectory.getTotalTimeSeconds();
    states = m_trajectory.getStates().size();
    intervals = (int) (runtime / 0.02);
    Pose2d p = m_trajectory.getInitialPose();
    System.out.format("runtime:%-3.1f number states:%d intervals:%d\n",runtime,states,intervals); 
   // m_drive.resetOdometry();

    //System.out.println(p);

    m_timer.reset();
    m_timer.start();
    
    //m_drive.startAuto();
    elapsed=0;
  
    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }


  Trajectory getTrajectory() {
      return pathPlannerTest();
  }

  Trajectory pathPlannerTest() {
    try {
      String file=m_drive.centerPosition()?"Center":"NotCenter";
      PathPlannerTrajectory trajectory = PathPlanner.loadPath(file, 
        new PathConstraints(Drivetrain.kMaxVelocity,Drivetrain.kMaxAcceleration)); // max vel & accel

        System.out.println("selecting auto path:"+file);
    
      Pose2d p0 = trajectory.getInitialHolonomicPose();

      // Pathplanner sets 0,0 as the lower left hand corner (FRC field coord system) 
      // for Gazebo, need to subtract intitial pose from each state so that 0,0 is 
      // in the center of the field 

      List<State> states = trajectory.getStates();
      for(int i=0;i<states.size();i++){
        PathPlannerTrajectory.PathPlannerState state=trajectory.getState(i);
        Pose2d p=state.poseMeters;

        Rotation2d h=state.holonomicRotation;

        Pose2d pr=p.relativeTo(p0);
        //if(i==0)
       //  pr=new Pose2d(pr.getTranslation(),new Rotation2d()); // 
        state.holonomicRotation=h.plus(new Rotation2d(Math.toRadians(180))); // go backwards

        //Pose2d psi=state.poseMeters.relativeTo(p0);
        state.poseMeters=pr;
        //System.out.println(state.poseMeters);
      }
      return trajectory;
    } catch (Exception ex) {
      System.out.println("failed to create pathweaver trajectory");
      ex.printStackTrace();
      return null;
    }
  }
  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
    elapsed = m_timer.get();
    if(m_trajectory==null){
      System.out.print("ERROR DrivePath.execute - trajectory is null");
       return;
    }
    //elapsed = m_drive.getTime();
  
    Trajectory.State reference = m_trajectory.sample(elapsed);
    //System.out.format("Time:%-1.3f X Current X:%-1.2f Target:%-1.2f\n",
    //elapsed,m_drive.getPose().getX(),reference.poseMeters.getX()
   // );

    ChassisSpeeds speeds = m_ppcontroller.calculate(m_drive.getPose(), (PathPlannerState) reference);
      m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

  }

    // =================================================
  // end: Called once the command ends or is interrupted.
  // =================================================
  
  @Override
  public void end(boolean interrupted) {
    System.out.println("DRIVEPATH_END");
    if (m_trajectory == null)
      return;
    //m_drive.endAuto();
    //TagDetector.setBestTarget();

    //m_drive.reset();
    // m_drive.enable();
  }

  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_trajectory == null){
      System.out.println("Failed to Create Trajectory");
      return true;
    }
    return (elapsed >= 1.001 * runtime || m_trajectory == null);

    //||m_drive.disabled()
  }
}
