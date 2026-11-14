package org.firstinspires.ftc.teamcode.EK10582.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.EK10582.EKLinear;
import org.firstinspires.ftc.teamcode.EK10582.subsystem.Flower;
import org.firstinspires.ftc.teamcode.EK10582.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants;

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

            robot.turret.activeLaunch = driverStation.gamepad2.right_bumper;

            // turret target goal
            if(driverStation.gamepad2.dpad_up){
                robot.turret.currentTargetGoal = SubsystemConstants.TrackingGoal.REDFRONT;
                }
            else if (driverStation.gamepad2.dpad_down){
                robot.turret.currentTargetGoal = SubsystemConstants.TrackingGoal.REDBACK;
            }
            else if(driverStation.gamepad2.dpad_left){
                robot.turret.currentTargetGoal = SubsystemConstants.TrackingGoal.BLUEBACK;
            }
            else if(driverStation.gamepad2.dpad_right){
                robot.turret.currentTargetGoal = SubsystemConstants.TrackingGoal.BLUEFRONT;
            }

            robot.flower.up = driverStation.gamepad2.x;
            robot.flower.down = driverStation.gamepad2.a;
            robot.flower.medium = driverStation.gamepad2.y;
            robot.transfer.activeTransfer= driverStation.gamepad1.right_bumper;
            robot.transfer.lastBall= driverStation.gamepad1.left_bumper;


            telemetry.addData("Reset: ", driverStation.getReset());
            telemetry.addData("rawRB", gamepad2.right_bumper);
            robot.update();

        }
    }
}

