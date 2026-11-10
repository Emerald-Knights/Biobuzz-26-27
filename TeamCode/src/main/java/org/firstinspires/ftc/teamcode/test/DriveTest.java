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
public class DriveTest extends EKLinear {


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        Robot.getInstance().leftFront.setPower(0.8);
        Robot.getInstance().rightFront.setPower(0.8);
        Robot.getInstance().leftBack.setPower(0.8);
        Robot.getInstance().rightBack.setPower(0.8);

        if(timer.seconds() >= 7){
            Robot.getInstance().leftFront.setPower(0);
            Robot.getInstance().rightFront.setPower(0);
            Robot.getInstance().leftBack.setPower(0);
            Robot.getInstance().rightBack.setPower(0);

        }



        while(opModeIsActive()) {

//            motor2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

            telemetry.addData("motor speed: ", gamepad1.right_trigger-gamepad1.left_trigger);
//            telemetry.addData("position: ", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
