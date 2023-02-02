// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.TargetMgr.TagTarget;


public class Limelight extends Thread {
  final static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  final static NetworkTable table = inst.getTable("limelight");
  final NetworkTableEntry botPoseNTE = table.getEntry("botPose");
  final NetworkTableEntry tidNTE = table.getEntry("tid");

  public static final int None = 0;
  public static final int April = 1;
  public static final int Box = 2;
  public static final int Cone = 3;
  public static final int Post = 4;
  public static boolean haveTarget = false;

  static int currentMode = None;

  double botPose[] = botPoseNTE.getDoubleArray(new double[0]);
  double tid = tidNTE.getDouble(0);
  public static Pose3d botCoords;
  public static Pose3d tagCoords;
  public int tagID;

  /** Creates a new LimeLight. */
  public Limelight() {

  }

  public void run() {
    // do something
    while (!Thread.interrupted()) {
      try {
        Thread.sleep(20);
        switch (currentMode) {
          default:
          case None:
            notargets();
            break;
          case Box:
            getBoxTarget();
            break;
          case Cone:
            getConeTarget();
            break;
          case April:
            getAprilTarget();
            break;
          case Post:
            break;

        }
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  private void getAprilTarget() {
    haveTarget = true;
  }

  private void getConeTarget() {
    haveTarget = true;
  }

  private void getBoxTarget() {
    haveTarget = true;
  }

  private void notargets() {
    haveTarget = false;
  }

  public Pose3d updateBotPose(double[] p) {
    return new Pose3d(p[0], p[1], p[2], new Rotation3d(p[3], p[4], p[5]));

  }

  public double updateApril() {
    return tid;
  }

  void calculateApril() {
    botCoords = updateBotPose(botPose);
    tagID = (int) updateApril() - 1;
    ArrayList<TagTarget> temp = TargetMgr.targets;
    tagCoords = temp.get(tagID).getPose();
  }

  public static void setMode(int m) {
    currentMode = m;
    table.getEntry("pipeline").setNumber(m);
    System.out.println("mode: " + m);

  }

  public void setPose(double[] p) {
    botCoords = updateBotPose(p);
  }

  public Pose2d getPose2d() {
    Pose2d pose = null;
    if (!haveTarget || currentMode != April)
      return pose;
      double d[] = inst.getTable("limelight").getEntry("botPose").getDoubleArray(new double[6]);
      pose = new Pose2d(d[0], d[1], new Rotation2d (d[5]));
      return pose;

  }

}
