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
  private static final int driverCam = 0;
  private static final int looking = 1;
  private static final int targetFound = 2;
  private static final int placement = 3;
  private int count = 0;
  /** Creates a new DriveToTarget. */
  PIDController turnController = new PIDController(.1,0,0);
  PIDController areaController = new PIDController(.1,0,0);

  Timer m_timer = new Timer();

  private static int currentMode;

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
    switch (currentMode) {
      case driverCam:
        Limelight.setMode(Limelight.None);
      case looking:
        count++;
        Limelight.setMode(Limelight.Cone);
        if (count == 10) {
        Limelight.setMode(Limelight.Box);
        } else if (count == 20) {
          Limelight.setMode(Limelight.Cone);
          count = 0;
        }
        if (Limelight.haveTarget) {
          currentMode = targetFound;
        }
      case targetFound:
        m_drive.drive(areaCorrection, 0, offsetCorrection, false);
        //TODO put arm stuff here
        if (turnController.atSetpoint()/* && arm.atSetpoint() */) {
          currentMode = placement;
          Limelight.setMode(Limelight.None);
        }
      case placement:
        Limelight.setMode(Limelight.April);

        currentMode = driverCam;
    }
  }

  public static void setMode(int mode) {
    currentMode = mode;
  } 
  public static int getMode() {
    return currentMode;
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