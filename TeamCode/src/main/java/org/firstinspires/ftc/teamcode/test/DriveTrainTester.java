package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.EK10582.subsystem.Robot.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.EK10582.subsystem.Robot;
import org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants;
import org.firstinspires.ftc.teamcode.EK10582.teleop.DriverStation;

@TeleOp(name="Drive Train Tester")
public class DriveTrainTester extends LinearOpMode {
    public Robot robot = Robot.getInstance();
    public Telemetry allTelemetry;
    public DriverStation driverStation;

    @Override
    public void runOpMode() {

        driverStation = new DriverStation(gamepad1, gamepad2);

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        double slowMode = 0.5;
        double ratio;
        double speed = SubsystemConstants.SPEED;


        while(opModeIsActive()) {

            double lx = -driverStation.getLeftStickX();
            double ly = driverStation.getLeftStickY();
            double rx = driverStation.getRightStickX();

            //slowmode

            if (driverStation.getLB1()){
                if (slowMode == 0.5)
                    slowMode = 0.8;
                else
                    slowMode = 0.5;
            }

            //Set motor powers to linear combination of controller input vectors
            double lf = ly + lx + rx;
            double rf = ly - lx - rx;
            double lb = ly - lx + rx;
            double rb = ly + lx - rx;

            //Scales the values so none of them exceeds one while still maintaining max possible speed
            double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            //Avoids dividing by 0
            if (max == 0) {
                ratio = 0;
            }
            //If magnitude is greater than 1 divide by max speed
            else if (magnitude > 1){
                ratio = 1/max;
            }
            //If magnitude is less than 1 scale by magnitude to maintain lower speeds
            else {
                ratio = magnitude / max * speed;
            }

            leftFront.setPower(lf*ratio*slowMode);
            rightFront.setPower(rf*ratio*slowMode);
            leftBack.setPower(lb*ratio*slowMode);
            rightBack.setPower(rb*ratio*slowMode);

//        robot.mecanumDrive.update();
//
            telemetry.addData("lx (strafe)", lx);
            telemetry.addData("ly (forward)", ly);
            telemetry.addData("rx (turn)", rx);
            telemetry.update();

        }
    }
}
