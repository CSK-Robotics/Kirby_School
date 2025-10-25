package frc.robot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Autonomous {
    private Limelight camera_object;
    private int targetAprilTag;
    public Autonomous(Limelight camera, DifferentialDrive driveTrain, int targetAprilTag) {
        camera_object = camera;
        camera_object.turnOnLEDS();
        this.targetAprilTag = targetAprilTag;
    }

    public void tick() {
        camera_object.getAprilTagPosition();
    }
}