// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetection;
import static frc.robot.Constants.*;

public class DetectorAprilTag extends Thread {
  /** Creates a new AprilTagDetector. */
  public Camera m_camera;

  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }
  

  static Mat mat;
  static Mat graymat = new Mat();
  protected AprilTagDetector wpi_detector;

  public double h_FOV = 50;
  public double aspect = ((double)kImageWidth ) / kImageHeight;
  public double v_FOV = 50;
  public double cx = kImageWidth/2;
  public double cy = kImageHeight/2;
  public double fx = cx / Math.tan(0.05*Math.toRadians(h_FOV));
  public double fy = cy / Math.tan(0.05*Math.toRadians(v_FOV));

  public static double targetSize = Units.inchesToMeters(6);


  public DetectorAprilTag(Camera camera) {
    wpi_detector = new AprilTagDetector();
    wpi_detector.addFamily("tag16h5", 0);
    m_camera.camera = CameraServer.startAutomaticCapture(0);
    camera = m_camera;
  }

  // private DetectionResult[] getTags(Mat mat) { 
  //   Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
  //   DetectionResult[] result = AprilTagJNI.aprilTagDetect(wpi_detector, mat, true, targetSize, fx, fy, cx, cy, 1);
  //   return result;
  // }

  private Mat getUsbCameraFrame() {
    Mat mat = new Mat();
    m_camera.UsbCameraSink.grabFrame(mat);
    return mat;
  }

  public void putFrame(CvSource source, Mat m) {
    if (m != null) {
      source.putFrame(m);
    }
  }
  @Override
  public void run() {
    while (true) {
    try {
      Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
      Thread.sleep(50);

      m_camera.UsbCameraSink.grabFrame(mat);
      AprilTagDetection[] detections = wpi_detector.detect(graymat);
      for (AprilTagDetection detection : detections) {
        // draw lines around the tag
        for (var i = 0; i <= 3; i++) {
          var outlineColor = new Scalar(0, 255, 0);
          var h = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(h), detection.getCornerY(h));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }
        var crossColor = new Scalar(0, 0, 255);
        Imgproc.putText(
          mat,
          Integer.toString(detection.getId()),
          new Point(cx + 10, cy),
          Imgproc.FONT_HERSHEY_SIMPLEX,
          1,
          crossColor,
          3);
      }
      putFrame(Camera.source, mat);

    } catch (Exception e) {
      System.out.println("apriltag exception: " + e);
    }
  }
}

}
