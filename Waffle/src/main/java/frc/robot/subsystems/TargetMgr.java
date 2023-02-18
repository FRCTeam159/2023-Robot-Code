// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TargetMgr extends SubsystemBase {

  public static ArrayList<TagTarget> targets=new ArrayList<>();

  static final public  int NO_TAGS=0;
  static final public  int SINGLE_TAG=1;
  static final public  int FRONT_TAGS=2;
  static final public  int PERIMETER_TAGS=3;
  static final public  int FIELD_TAGS=4;

  public static Translation3d field_center_offset=new Translation3d(325.4,157,0); 

  public static Translation3d robot_offset=new Translation3d(0,0,0);
  public static Rotation3d robot_rotation=new Rotation3d(0,0,0);
  public static Translation3d field_center=field_center_offset.times(Units.inchesToMeters(1));

  public static double targetSize=0.1524;

  static int type=FIELD_TAGS;

  static void setFieldTargets(){
    // data taken from inset in field drawing 5 of 7

    Translation3d tags[]={
    new Translation3d(610.77,42.19,18.22), // 1 180
    new Translation3d(610.77,108.19,18.22),// 2 180
    new Translation3d(610.77,174.19,18.22),// 3 180 - drawing chart bug (says y=147)
    new Translation3d(636.96,265.74,27.38),// 4 180
    new Translation3d(14.45,265.74,27.38), // 5 0
    new Translation3d(40.25,174.19,18.22), // 6 0   - drawing chart bug (says y=147)
    new Translation3d(40.25,108.19,18.22), // 7 0
    new Translation3d(40.25,42.19,18.22)   // 8 0
    };

    for(int i=0;i<tags.length;i++)
        tags[i]=tags[i].times(Units.inchesToMeters(1)).minus(field_center);



    // frc has 0,0 at lower left corner but for Gazebo 0,0 is center of field
    // for display and simulation translate tags to field center

    System.out.println("Robot:"+robot_offset);

    for(int i=0;i<tags.length;i++){
        Translation3d tag=robot_offset.minus(tags[i]);
        double x=tag.getX();
        double y=tag.getY();
        double z=tag.getZ();
        double tag_angle=i>3?Math.toRadians(180):0; // looking forward
        Rotation3d tag_rotation=new Rotation3d(0,0,tag_angle);
        Rotation3d robot_angle=robot_rotation.rotateBy(tag_rotation);
        TagTarget tt=new TagTarget(i+1,x,y,z,robot_angle.getZ());
        targets.add(tt); 
    }
    System.out.println("tag offsets from FRC origin");
    for(int i=0;i<targets.size();i++){
        Translation3d tag=robotToFRC(targets.get(i).getTranslation());
        System.out.println("id:"+(i+1)+" "+getString(tag));
    }
    System.out.println("\ntag offsets to robot");
    for(int i=0;i<targets.size();i++)
        System.out.println(targets.get(i));

    System.out.println("\noffsets from field center");
    System.out.println("robot "+getString(robot_offset.minus(field_center)));
    for(int i=0;i<targets.size();i++){
        Translation3d tag_frc=robotToFRC(targets.get(i).getTranslation());
        Translation3d tag_center=tag_frc.minus(field_center);    // offsets to gazebo origin
        System.out.println("id:"+(i+1)+" "+getString(tag_center));
    }
}

static String getString(Translation3d t){
    return String.format("x:%-2.2f y:%-2.2f z:%-2.2f",t.getX(),t.getY(),t.getZ());
}
static Translation3d robotToFRC(Translation3d loc){
    return robot_offset.minus(loc);
}
static Translation2d robotToFRC(Translation2d loc, int tag_id){
    TagTarget target=getTarget(tag_id);
    Translation3d tag=robotToFRC(target.getTranslation());
    Translation2d pos=tag.toTranslation2d().plus(loc);
    //System.out.println(tag_id+" "+pos+" "+tag+" "+loc);

    return pos;
}

  /** Creates a new TargetMgr. */
  public TargetMgr() {
    setFieldTargets();
  }

  static public TagTarget getTarget(int i){
    int id=(type==FIELD_TAGS)?i-1:i;
    if(id<0 || id >targets.size())
        return null;
    //System.out.println(i+" "+id+" "+type);
    return targets.get(id);
}

  static public class TagTarget {
    Pose3d targetPose;
    int targetID=0;

    public TagTarget( int id, Pose3d pose) {
        targetPose = pose;
        targetID = id;
    }

    public TagTarget(int id, double x, double y, double z, double a) {
        targetPose = new Pose3d(x,y,z,new Rotation3d(0,0,a));
        targetID = id;
    }
    
    public int getID(){ 
        return targetID;
    }
    public Pose3d getPose(){
        return targetPose;
    }
    public Translation3d getTranslation(){
        return targetPose.getTranslation();
    }
    public double getTargetSize(){
        return targetSize;
    }

    public TagTarget moveTo(Translation3d loc){
        Translation3d trans=loc.minus(targetPose.getTranslation());
        Pose3d pose=new Pose3d(trans,targetPose.getRotation());
        return new TagTarget(targetID,pose);
    }
    public String toString(){
        double angle=targetPose.getRotation().toRotation2d().getDegrees();
        return String.format("id:%d x:%-2.1f y:%2.1f z:%1.2f a:%3.1f",
        targetID,targetPose.getX(),targetPose.getY(),targetPose.getZ(),angle);
    }
   
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
