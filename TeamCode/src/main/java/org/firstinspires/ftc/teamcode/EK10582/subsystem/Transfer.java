package org.firstinspires.ftc.teamcode.EK10582.subsystem;

import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.GOPOSITION;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.PUSHPOSITION;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.STOPPOSITION;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer extends Subsystem{
    public enum State {
        STOP,
        GO,
        PUSH
    }

    public State state = State.STOP;

    private double targetPosition = Double.NaN;

    public void cycleState() {
        switch (state) {
            case STOP:
                state = State.GO;
                break;
            case GO:
                state = State.PUSH;
                break;
            case PUSH:
                state = State.STOP;
                break;
        }
    }
    public void init(boolean isAuton) {
        state = state.STOP;
        targetPosition = Double.NaN;
    }

    public void update(boolean isAuton) {

        double position = 0.0;
        switch (state) {
            case STOP:
                Robot.getInstance().transferServo.setPosition(GOPOSITION);
                break;
            case GO:
                Robot.getInstance().transferServo.setPosition(PUSHPOSITION);
                break;
            case PUSH:
                Robot.getInstance().transferServo.setPosition(STOPPOSITION);
            default:
                position = STOPPOSITION;
                break;
        }

        if (position != targetPosition) {
            targetPosition = position;
            Robot.getInstance().transferServo.setPosition(position);
        }
    }

    public void stop() {
        Robot.getInstance().transferServo.setPosition(STOPPOSITION);
    }

    public void printToTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer State: ", Robot.getInstance().transferServo.getPosition());
    }

}






