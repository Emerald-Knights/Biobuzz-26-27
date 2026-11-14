package org.firstinspires.ftc.teamcode.EK10582.subsystem;

import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.HIGH;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.LOW;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.REST;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Transfer extends Subsystem{
    public boolean activeTransfer;
    public boolean lastBall;

    public void init(boolean isAuton) {
        activeTransfer = false;
    }

    public void update(boolean isAuton) {
        if (activeTransfer) {
            Robot.getInstance().transferServo.setPosition(LOW);
        } else if (lastBall) {
            Robot.getInstance().transferServo.setPosition(HIGH);
        } else {
            Robot.getInstance().transferServo.setPosition(REST);
        }
    }

    public void stop() {
    }

    public void printToTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer State: ", Robot.getInstance().transferServo.getPosition());
    }
}


