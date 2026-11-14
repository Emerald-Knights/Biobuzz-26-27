package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Flywheel Tester")
public class FlywheelTester extends LinearOpMode {

    public boolean morePower = gamepad1.yWasPressed();
    public boolean lessPower = gamepad1.aWasPressed();


    @Override
    public void runOpMode() {

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "flywheel");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double power = 0;

        waitForStart();
//
        while (opModeIsActive()) {

            if (morePower) {
                power += .05;
            }

            if (lessPower) {
                power -= .05;
            }

            motor.setPower(power);

            telemetry.addData("Motor power", power);
            telemetry.update();
        }
    }
}
