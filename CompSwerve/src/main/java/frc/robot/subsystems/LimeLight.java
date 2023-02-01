// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import javax.swing.Box;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.TargetMgr.TagTarget;
import static frc.robot.Constants.*;

public class Limelight extends Thread {
  final static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  final static NetworkTable limelightTable = inst.getTable("limelight");
  final NetworkTableEntry botposeNTE = limelightTable.getEntry("botpose");
  final NetworkTableEntry tidNTE = limelightTable.getEntry("tid");
  final static NetworkTableEntry txNTE = limelightTable.getEntry("tx");
  final NetworkTableEntry tyNTE = limelightTable.getEntry("ty");
  final NetworkTableEntry taNTE = limelightTable.getEntry("ta");

  public static final int None = 5;
  public static final int April = 0;
  public static final int Box = 1;
  public static final int Cone = 2;
  public static final int Post = 3;
  public static boolean haveTarget = false;

  public static int currentMode = Box;

  double botpose[] = botposeNTE.getDoubleArray(new double[6]);
  double tid = tidNTE.getDouble(-1);
  public static Pose3d botCoords;
  public static Pose3d tagCoords;
  public int tagID;
  public static double tx;
  public static double ty;
  public static double ta;
  public static double targetArea;


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
            getPostTarget();
            break;

        }
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  private void getAprilTarget() {
    botpose = botposeNTE.getDoubleArray(new double[-1]);
    if (botpose.length != 0) {
      haveTarget = true;
    }
  }

  private void getConeTarget() {
    tx = txNTE.getDouble(-1);
    if (tx != -1) {
      ta = taNTE.getDouble(-1);
      targetArea = kConeTargetArea;
      haveTarget = true;
    }
  }

  private void getBoxTarget() {
  tx = txNTE.getDouble(-1);
  if (tx != -1) {
    ta = taNTE.getDouble(-1);
    targetArea = kBoxTargetArea;
    haveTarget = true;
  }
  }
  private void getPostTarget() {
    if (tx != -1) {
      ta = taNTE.getDouble(-1);
      targetArea = kPostTargetArea;
      haveTarget = true;
    }
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
    botCoords = updateBotPose(botpose);
    tagID = (int) updateApril() - 1;
    ArrayList<TagTarget> temp = TargetMgr.targets;
    tagCoords = temp.get(tagID).getPose();
  }

  public static void setMode(int m) {
    currentMode = m;
    limelightTable.getEntry("pipeline").setNumber(m);
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
