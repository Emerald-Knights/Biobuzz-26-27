package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Drive Test")
public class DriveTest extends LinearOpMode {

    private static final double RUN_POWER = 0.8;
    private static final double RUN_SECONDS = 7;

    @Override
    public void runOpMode() {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
          //  if (timer.seconds() < RUN_SECONDS) {
//                leftFront.setPower(RUN_POWER);
//                leftBack.setPower(RUN_POWER);
//                rightFront.setPower(RUN_POWER);
//                rightBack.setPower(RUN_POWER);
//            } else {
//                leftFront.setPower(0);
//                leftBack.setPower(0);
//                rightFront.setPower(0);
//                rightBack.setPower(0);
//            }

            //telemetry.addData("time", "%.1f s", timer.seconds());
            telemetry.addData("leftFront pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
    }
}
