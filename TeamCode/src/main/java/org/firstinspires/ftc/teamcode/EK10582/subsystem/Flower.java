package org.firstinspires.ftc.teamcode.EK10582.subsystem;

import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.FLOWER_DOWN;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.FLOWER_MEDIUM;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.FLOWER_UP;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Driver cycles from Down -> medium -> up -> down with cycle state
 * Not used in auton
 */
public class Flower extends Subsystem {
    public enum State {
        UP,
        MEDIUM,
        DOWN
    }

    public State state = State.MEDIUM;

    private double targetPosition = Double.NaN;

    public void cycleState() {
        switch (state) {
            case DOWN:
                state = State.MEDIUM;
                break;
            case MEDIUM:
                state = State.UP;
                break;
            case UP:
                state = State.DOWN;
                break;
        }
    }

    public void init(boolean isAuton) {
        state = State.MEDIUM;
        targetPosition = Double.NaN;
    }

    public void update(boolean isAuton) {
        if (isAuton)
            return;


        double position;
        switch (state) {
            case DOWN:
                position = FLOWER_DOWN;
                break;
            case UP:
                position = FLOWER_UP;
                break;
            case MEDIUM:
            default:
                position = FLOWER_MEDIUM;
                break;
        }

        // Only send a new command when the target position actually changes
        if (position != targetPosition) {
            targetPosition = position;
            Robot.getInstance().flowerServo.setPosition(position);
        }
    }

    public void stop() {
        Robot.getInstance().flowerServo.setPosition(FLOWER_UP);
    }

    public void printToTelemetry(Telemetry telemetry) {
        telemetry.addData("Flower State: ", state);
        telemetry.addData("Flower Target: ", targetPosition);
    }
}
