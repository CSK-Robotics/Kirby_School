package frc.robot;
import java.util.Dictionary;
import java.util.Hashtable;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class Limelight {
    private int pixelOffsetTolerance = 10;
    private float degreeOffsetTolerance = 5;
    private int imageWidth = 300;
    private int imageHeight = 500;
    private int centerX = imageHeight/2;
    private int centerY = imageWidth/2;

    public Limelight(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex("", pipelineIndex);
    }

    public void turnOffLEDS() {
        //LimelightHelpers.setLEDMode_ForceOff("");
    }

    public void turnOnLEDS() {
        //LimelightHelpers.setLEDMode_ForceOn("");
    }

    public Dictionary getAprilTagPosition() {
        Dictionary<String, Integer> d = new Hashtable<>();
        /*
        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
        double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
        boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?
        System.out.println("hasTarget: " + hasTarget);
        */
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id;                    // Tag ID
            double txnc = fiducial.txnc;             // X offset (no crosshair)
            double tync = fiducial.tync;             // Y offset (no crosshair)
            double ta = fiducial.ta;                 // Target area
            double distToCamera = fiducial.distToCamera;  // Distance to camera
            double distToRobot = fiducial.distToRobot;    // Distance to robot
            double ambiguity = fiducial.ambiguity;   // Tag pose ambiguity

            System.out.println("April Tag: " + id);
        }
        return d;
    }

    public void updateAprilTarget(int newTargetID) {
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{newTargetID}); // Only track these tag IDs
    }
}
