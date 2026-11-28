package org.firstinspires.ftc.teamcode.EK10582.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.EK10582.auton.action.AutonFlowerDown;
import org.firstinspires.ftc.teamcode.EK10582.auton.action.AutonFlowerUp;
import org.firstinspires.ftc.teamcode.EK10582.auton.action.AutonIntake;
import org.firstinspires.ftc.teamcode.EK10582.auton.action.AutonShoot;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Autonomous
public class FrontTwo extends AutonBase{

    protected PoseFactory poseFactory() {
        return PoseFactory.degrees();
    }

    private final PoseFactory frontTwo = poseFactory();
    // our poses
    private final Pose startPose = frontTwo.of(56, 8, 180);

    //private final Pose garden = frontTwo.of(36, 8, 180);
    private final Pose gardenComplete = frontTwo.of(8, 8, 180);
    private final Pose flower2 = frontTwo.of(47, 121, 90);
    private final Pose flower2Complete = frontTwo.of(47, 129, 90);
    private final Pose flower3 = frontTwo.of(18, 47, 180);
    private final Pose flower3Complete = frontTwo.of(13, 47, 180);
    private final Pose insurance = frontTwo.of(36, 110, 180);
    private final Pose park = frontTwo.of(9, 95, 180);


    //private final Pose controlPose2 = frontTwo.of(45, 27, 0);
    private final Pose controlPose1 = frontTwo.of(31, 110, 0);


    private Path garden() {
        return line(startPose, gardenComplete).linear(startPose, gardenComplete);

    }
    private Path shoot1() {
        return curve(gardenComplete, controlPose1, flower2).linear(gardenComplete, flower2);

    }
    private Path flower2in() {
        return line(flower2, flower2Complete).linear(flower2, flower2Complete);

    }
    private Path flower2out() {
        return line(flower2Complete, flower2).linear(flower2Complete, flower2);

    }
    private Path shoot2() {
        return line(flower2, flower3).linear(flower2Complete, flower3);

    }
    private Path flower3in() {
        return line(flower3, flower3Complete).linear(flower3, flower3Complete);

    }
    private Path flower3out() {
        return line(flower3Complete, flower3).linear(flower3Complete, flower3);
    }
    private Path insurance() {
        return line(flower3, insurance).linear(flower3, insurance);
    }
    private Path park() {
        return line(insurance, park).linear(insurance, park);
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
        run(sequential(
                action(new AutonFlowerUp()),
                //intake while moving
                follow(follower, garden()),
                follow(follower, shoot1()),
                action(new AutonShoot()),
                action(new AutonFlowerDown()),
                follow(follower, flower2in()),
                action(new AutonIntake()),
                follow(follower, flower2out()),
                action(new AutonFlowerUp()),
                action(new AutonShoot()),
                follow(follower, shoot2()),
                action(new AutonFlowerDown()),
                follow(follower, flower3in()),
                action(new AutonIntake()),
                follow(follower, flower3out()),
                action(new AutonFlowerUp()),
                follow(follower, insurance()),
                action(new AutonShoot()),
                follow(follower, park()),
                waitMs(2000)
        ));
    }
}
