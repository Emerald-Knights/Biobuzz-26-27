package org.firstinspires.ftc.teamcode.EK10582.auton.action;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight extends Action {
    //this is an abstract parent class for all of the actions the robot will do.
    //some examples of actions are: moveArm, collectPixel, dropPixel, etc.
    public boolean isComplete = false;
    private double distance;
    private double AprilTagFar = 0;
    public boolean farEnough = false;
    Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
    LLResult llResult = limelight.getLatestResult();

    public boolean distanceTracker() {
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTags(llResult.getTa());
        }

        return distance < AprilTagFar;
    }

    public double getDistanceFromTags(double ta) {
        double scale = 176.9951;
        double power = -0.5621096;
        double distance = scale * Math.pow(ta, power);
        return distance;
    }

    @Override
    public void start() {
        isComplete = false;

    }

    @Override
    public void update() {
        if (distanceTracker()) {
            farEnough = true;
        }

    }

    @Override
    public void end() {
        isComplete = true;
    }

}

