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
    private double targetAreaThreshold = 50; // Area threshold (how big the aprilTag should be) 
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

    public void moveForwardTick() {
        // Move forward while adjusting to zone in on target:
        double currentVelocity = 0;
    }

    public void setAprilTarget(int aprilTarget) {
        this.targetAprilTag = aprilTarget;
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
                rightController.setReference(rightFFSpeed, ControlType.kVelocity);
                leftController.setReference(leftFFSpeed, ControlType.kVelocity);
            }
            else {
                rightController.setReference(0, ControlType.kVelocity);
                leftController.setReference(0, ControlType.kVelocity);
            }
        }
    }
}