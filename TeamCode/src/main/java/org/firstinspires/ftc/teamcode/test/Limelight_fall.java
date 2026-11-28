package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.navigation.Orientation.getOrientation;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.hardware.IMU;
@Autonomous(name = "Limelight_fall", group = "Examples")
public class Limelight_fall extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    private double tagSeenSince = -1; // April tag is not seen
    private double tagLastSeen = -1;    // last time that the april tag was actually visible

    private boolean canShoot = false;

    private static final double grace = 0.15;  // limelight dropout tollerance

    private double distance;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); //april tag number 24
        imu=hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD); //change based on orientation of robot once assembled
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    @Override
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        boolean tagVisible = llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty();
        double now = getRuntime();

        if(tagVisible){
            if (tagSeenSince < 0){
                tagSeenSince = now; // starting the timer of seeing the tag
            }
            tagLastSeen = now;
        } else if (tagSeenSince >= 0 && now - tagLastSeen > grace){
            tagSeenSince = -1;
        }

        canShoot = tagSeenSince >= 0 && (now - tagSeenSince) > 0.75;


        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTags(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            if (botPose != null) {
                telemetry.addData("BotPose", botPose.toString());
                telemetry.addData("Heading/Yaw", botPose.getOrientation().getYaw());
            }
        }

        telemetry.addData("Can shoot:", canShoot);

    }

    public double getDistanceFromTags(double ta) {
        double scale = 176.9951;
        double power = -0.5621096;
        double distance = scale * Math.pow(ta, power);
        return distance;
    }

    public void stop(){
        limelight.stop();
    }
}
