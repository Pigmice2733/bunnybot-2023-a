// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConfig;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonTrackedTarget currentTarget;

    public Vision() {
        camera = new PhotonCamera("HD_Pro_Webcam_C920");

        ShuffleboardHelper.addOutput("Yaw", Constants.VISION_TAB,
                () -> currentTarget == null ? 0 : currentTarget.getYaw()).asDial(-180, 180);
        ShuffleboardHelper.addOutput("Pitch", Constants.VISION_TAB,
                () -> currentTarget == null ? 0 : currentTarget.getPitch()).asDial(-180, 180);
        ShuffleboardHelper.addOutput("Skew", Constants.VISION_TAB,
                () -> currentTarget == null ? 0 : currentTarget.getSkew()).asDial(-180, 180);
        ShuffleboardHelper.addOutput("Area", Constants.VISION_TAB,
                () -> currentTarget == null ? 0 : currentTarget.getArea()).asDial(0, 1);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult results = camera.getLatestResult();
        if (!results.hasTargets()) {
            currentTarget = null;
            return;
        }

        currentTarget = results.getBestTarget();
    }

    public PhotonTrackedTarget getCurrentTarget() {
        return currentTarget;
    }

    /** Returns the distance between the camera and the current target, in meters. */
    public double distanceToTarget() {
        return (VisionConfig.TARGET_HEIGHT_METERS - VisionConfig.CAMERA_HEIGHT_METERS)
                / Math.tan(currentTarget.getPitch());
    }
}
