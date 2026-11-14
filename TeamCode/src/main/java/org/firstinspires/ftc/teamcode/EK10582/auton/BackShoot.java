package org.firstinspires.ftc.teamcode.EK10582.auton;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.EK10582.auton.action.AutonFlowerUp;
import org.firstinspires.ftc.teamcode.EK10582.auton.action.AutonShoot;
import org.firstinspires.ftc.teamcode.EK10582.auton.action.Limelight;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.waitMs;

@Autonomous
public class BackShoot extends AutonBase {

    protected PoseFactory poseFactory() {
        return PoseFactory.degrees();
    }

    private final PoseFactory backShoot = poseFactory();
    // our poses
    private final Pose startPose = backShoot.of(62, 132, 90);

    private final Pose shoot = backShoot.of(62, 115, 90);
    private final Pose park = backShoot.of(12, 115, 90);


    private Path shoot() {
        return line(startPose, shoot).linear(startPose, shoot);
    }
    private Path park() {
        return line(shoot, park).linear(shoot, park);

    }

    @Override
    protected Pose startPose() {
        return startPose;
    }
    @Override
    protected void autonTelemetry() {
        allTelemetry.addData("X", follower.pose().x());
        allTelemetry.addData("Y", follower.pose().y());
        allTelemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        allTelemetry.addData("Follower Mode", follower.mode());
    }


    @Override
    public void runOpMode() {
        waitForStart(); // creates the follower at startPose()

        Limelight limelightAction = new Limelight();

        run(
                parallel(
                        action(limelightAction),

                        sequential(
                                action(new AutonFlowerUp()),
                                conditional(
                                        limelightAction::distanceTracker,

                                        sequential(
                                                follow(follower, shoot()),
                                                action(new AutonShoot())
                                        ),

                                        // Otherwise
                                        waitMs(2000)
                                ),

                                follow(follower, park()),
                                waitMs(2000)
                        )
                )
        );
    }
}

