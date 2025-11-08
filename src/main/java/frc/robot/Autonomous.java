package frc.robot;
import java.util.Dictionary;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Autonomous {
    private Limelight camera_object;
    private int targetAprilTag;
    private double leftFFSpeed = 50;
    private double rightFFSpeed = 50;
    private double targetXTolerance = 0.5; // Tolerance of camera being zeroed on aprilTag
    private double targetAreaThreshold = 0.07; // Area threshold (how big the aprilTag should be) 
    private double targetAreaTolerance = 0.5;
    private double slowSpinCurrentAngle = 0.0;
    private SparkClosedLoopController leftController;
    private SparkClosedLoopController rightController;

    public Autonomous(Limelight camera, SparkClosedLoopController rightController, SparkClosedLoopController leftController, int targetAprilTag) {
        camera_object = camera;
        camera_object.turnOnLEDS();
        this.targetAprilTag = targetAprilTag;
        this.leftController = leftController;
        this.rightController = rightController;
    }

    public void slowSpinTick() {
        // Spin clock-wise:
        if(this.slowSpinCurrentAngle < 360) {
            leftController.setReference(1, ControlType.kPosition);
            rightController.setReference(0, ControlType.kPosition);
            this.slowSpinCurrentAngle += 1;
        }
    }

    public double linearInterpolate(double x_target_offset) {
        double _slope = 0.05; // Tune this later
        double _intercept = 0.025;
        return Math.max(Math.min((_slope*Math.abs(x_target_offset)) + _intercept, 1.0), 0.0);
    }


    public void moveForwardTick(double x_target_offset) {
        // Move forward while adjusting to zone in on target:
        double currentVelocityL = this.leftFFSpeed;
        double currentVelocityR = this.rightFFSpeed;
        
        if(Math.abs(x_target_offset) <= this.targetXTolerance) {
            // Assume we're centered "enough"
            leftController.setReference(currentVelocityL, ControlType.kPosition);
            rightController.setReference(currentVelocityR, ControlType.kPosition);
        } else {
            double rampOffset = linearInterpolate(x_target_offset);
            System.out.println("Ramp: " + rampOffset);
            if(x_target_offset > 0) {
                // Target is off the right side
                currentVelocityR -= 20*rampOffset;
                leftController.setReference(currentVelocityL, ControlType.kPosition);
                rightController.setReference(currentVelocityR, ControlType.kPosition);
            } else {
                // Target is off the left side
                currentVelocityL -= 20*rampOffset;
                leftController.setReference(currentVelocityL, ControlType.kPosition);
                rightController.setReference(currentVelocityR, ControlType.kPosition);
            }
        }
    }

    public void setAprilTarget(int aprilTarget) {
        this.targetAprilTag = aprilTarget;
    }

    public boolean isCloseEnough(double targetArea) {
        if(targetArea > targetAreaThreshold) {
            return true;
        } else {
            return false;
        }
    }

    public void tick() {
        Dictionary<String, Double> bestAprilTag = camera_object.getAprilTagPosition();
        
        if( bestAprilTag.size() == 0 ) {
            // Stop current controller motion:
            rightController.setReference(0, ControlType.kVelocity);
            leftController.setReference(0, ControlType.kVelocity);
            
            // Do a slow spin tick to find target
            slowSpinTick();
        } else {
            if(bestAprilTag.get("id") == this.targetAprilTag) {
                // Reset slowSpingCurrentAngle for the next search
                slowSpinCurrentAngle = 0;
                // Move towards target:
                // rightController.setReference(rightFFSpeed, ControlType.kVelocity);
                // leftController.setReference(leftFFSpeed, ControlType.kVelocity);
                moveForwardTick(bestAprilTag.get("target_x"));
            }
            else {
                rightController.setReference(0, ControlType.kVelocity);
                leftController.setReference(0, ControlType.kVelocity);
            }
        }
    }
}