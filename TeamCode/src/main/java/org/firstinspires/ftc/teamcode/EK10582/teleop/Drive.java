package org.firstinspires.ftc.teamcode.EK10582.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.EK10582.EKLinear;
import org.firstinspires.ftc.teamcode.EK10582.subsystem.Flower;

@TeleOp(name="New Drive")
public class Drive extends EKLinear {

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            //drive
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

            robot.intake.activeIntake= driverStation.gamepad1.right_bumper;


            //driver 2 controls

            robot.intake.reverseIntake= driverStation.gamepad2.b;
            robot.launch.activeLaunch = driverStation.gamepad2.right_bumper;

            robot.flower.up = driverStation.gamepad2.x;
            robot.flower.down = driverStation.gamepad2.a;
            robot.flower.medium = driverStation.gamepad2.y;


            telemetry.addData("Reset: ", driverStation.getReset());
            telemetry.addData("rawRB", gamepad2.right_bumper);
            robot.update();

        }
    }
}

