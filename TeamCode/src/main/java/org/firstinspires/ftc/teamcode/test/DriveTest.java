package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EK10582.EKLinear;
import org.firstinspires.ftc.teamcode.EK10582.subsystem.Robot;

import java.sql.Time;

@TeleOp(name="Drive Test")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        ElapsedTime timer = new ElapsedTime();


        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx motor3 = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx motor4 = hardwareMap.get(DcMotorEx.class, "rightBack");


        while(opModeIsActive()) {


            motor.setPower(0.8);
            motor2.setPower(0.8);
            motor3.setPower(0.8);
            motor4.setPower(0.8);

            if(timer.seconds() >= 7){
                motor.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);
            }

           // telemetry.addData("motor speed: ", gamepad1.right_trigger-gamepad1.left_trigger);
//            telemetry.addData("position: ", motor.getCurrentPosition());
           // telemetry.update();
        }
    }
}
