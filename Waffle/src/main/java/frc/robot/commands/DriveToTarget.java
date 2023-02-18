// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class DriveToTarget extends CommandBase {
  /** Creates a new DriveToTarget. */
  PIDController turnController = new PIDController(.1,0,0);
  PIDController areaController = new PIDController(.1,0,0);

  Timer m_timer = new Timer();

  Drivetrain m_drive;
  public DriveToTarget(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    turnController.setTolerance(5, 5);
    areaController.setTolerance(5, 2);
    addRequirements(m_drive);
    System.out.println("constructor");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initialize");
    turnController.setSetpoint(0);
    areaController.setSetpoint(20);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offset = Limelight.tx;
    double area = Limelight.ta;
    double offsetCorrection = turnController.calculate(offset);
    double areaCorrection = areaController.calculate(area);
    System.out.println(Limelight.currentMode);
    if (Limelight.currentMode != 3) {
    m_drive.drive(areaCorrection, 0, offsetCorrection, false);
    } else {//not currently working
      System.out.println("visiontarget mode");

      m_drive.drive(0, offsetCorrection, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsedTime = m_timer.get();
    if (elapsedTime > 10) {
      System.out.println("timer ended");
      return true;
    }
    if (!Limelight.haveTarget) {
      System.out.println("no target");
      return true;
    }
    return (turnController.atSetpoint() && areaController.atSetpoint()) ; 
  }
}
