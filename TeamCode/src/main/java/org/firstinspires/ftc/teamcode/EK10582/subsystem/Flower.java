package org.firstinspires.ftc.teamcode.EK10582.subsystem;

import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.FLOWER_DOWN;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.FLOWER_MEDIUM;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.FLOWER_UP;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Single owner of the flower servo. Defaults to FLOWER_MEDIUM; "down" (intake) takes
 * priority over "up" if both are requested at once.
 */
public class Flower extends Subsystem {
    public enum State {
        UP,
        MEDIUM,
        DOWN
    }

    public State state = State.MEDIUM;

    public boolean down;
    public boolean up;
    public boolean medium;

    private double targetPosition = Double.NaN;

    public void init(boolean isAuton) {
        state = State.MEDIUM;
        down = false;
        up = false;
        medium = false;
        targetPosition = Double.NaN;
    }

    public void update(boolean isAuton) {
        if (down) {
            state = State.DOWN;
        } else if (up) {
            state = State.UP;
        } else if (medium) {
            state = State.MEDIUM;
        }

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

    }

    public void printToTelemetry(Telemetry telemetry) {
        telemetry.addData("Flower State: ", state);
        telemetry.addData("Flower Target: ", targetPosition);
    }
}
