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
        LimelightHelpers.setLEDMode_ForceOff("");
    }

    public void turnOnLEDS() {
        LimelightHelpers.setLEDMode_ForceOn("");
    }

    public Dictionary getAprilTagPosition() {
        Dictionary<String, Double> d = new Hashtable<>();
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        int bestAprilTagIndex = -1;
        int currentIndex = 0;
        double biggestArea = 0.0;
        
        for (RawFiducial fiducial : fiducials) {
            double ta = fiducial.ta;                 // Target area
            if(ta > biggestArea) {
                bestAprilTagIndex = currentIndex;
                biggestArea = ta;
            }
            currentIndex += 1;
        }

        if (bestAprilTagIndex != -1) {
            RawFiducial fiducial = fiducials[bestAprilTagIndex];

            int id = fiducial.id;                    // Tag ID
            double txnc = fiducial.txnc;             // X offset (no crosshair)
            double tync = fiducial.tync;             // Y offset (no crosshair)
            double ta = fiducial.ta;                 // Target area
            double distToCamera = fiducial.distToCamera;  // Distance to camera

            System.out.println("Best april tag ID: " + fiducials[bestAprilTagIndex].id + " index: " + bestAprilTagIndex + " target offset: " + txnc + " " + tync + " " + ta);

            d.put("id", (double)id);
            d.put("area", ta);
            d.put("distance", distToCamera);
            d.put("target_x", txnc);
            d.put("target_y", tync);
        }
        return d;
    }

    public void updateAprilTarget(int newTargetID) {
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{newTargetID}); // Only track these tag IDs
    }
}
