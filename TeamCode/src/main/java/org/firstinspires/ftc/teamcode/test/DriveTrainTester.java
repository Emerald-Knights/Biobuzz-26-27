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
import org.firstinspires.ftc.teamcode.EK10582.teleop.DriverStation;
import org.firstinspires.ftc.teamcode.EK10582.teleop.DriverStation;

@TeleOp(name="Drive Train Tester")
public class DriveTrainTester extends LinearOpMode {
    public Robot robot = Robot.getInstance();
    public Telemetry allTelemetry;
    public DriverStation driverStation;

    @Override
    public void runOpMode() {

        waitForStart();

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


        while(opModeIsActive()) {

            robot.mecanumDrive.lx = driverStation.getLeftStickX();
            robot.mecanumDrive.ly = driverStation.getLeftStickY();
            robot.mecanumDrive.rx = driverStation.getRightStickX();

            //slowmode

            if (driverStation.getLB1()){
                if (robot.mecanumDrive.slowMode == 0.5)
                    robot.mecanumDrive.slowMode = 0.8;
                else
                    robot.mecanumDrive.slowMode = 0.5;
            }

//        robot.mecanumDrive.update();
//
        telemetry.addData("lx (strafe)", robot.mecanumDrive.lx);
        telemetry.addData("ly (forward)", robot.mecanumDrive.ly);
        telemetry.addData("rx (turn)", robot.mecanumDrive.rx);
        telemetry.update();

        }
    }
}
