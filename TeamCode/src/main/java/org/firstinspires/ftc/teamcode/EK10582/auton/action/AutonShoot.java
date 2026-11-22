package org.firstinspires.ftc.teamcode.EK10582.auton.action;


//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.EK10582.subsystem.Robot;
import org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants;
//import org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants;


public class AutonShoot extends Action {


    private enum State {
        SHOOT,
        SHOOTEND,
        FINISHED
    }


    private State currentState = State.SHOOT;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime totalTimer = new ElapsedTime();


    @Override
    public void start() {
        totalTimer.reset();
        stateTimer.reset();
        setInternalState(State.SHOOT);
    }


    @Override
    public void update() {

        switch (currentState) {
            case SHOOT:
                //Robot.getInstance().launch.activeLaunch = true;
                Robot.getInstance().transferServo.setPosition(SubsystemConstants.GOPOSITION);
                Robot.getInstance().launchMotor.setVelocity(1500);
                Robot.getInstance().intakeMotor.setPower(0.8);
                if (stateTimer.milliseconds() >= 5000) {
                    setInternalState(State.SHOOTEND);
                }
                break;

            case SHOOTEND:
                //Robot.getInstance().launch.activeLaunch = true;
                Robot.getInstance().transferServo.setPosition(SubsystemConstants.PUSHPOSITION);
                Robot.getInstance().launchMotor.setVelocity(1500);
                Robot.getInstance().intakeMotor.setPower(0);

                if (stateTimer.milliseconds() >= 275) {
                    setInternalState(State.FINISHED);
                }
                break;


            case FINISHED:
                stopAll();
                isComplete = true;
                break;
        }
    }


    private void setInternalState(State newState) {
        if (currentState != newState) {
            currentState = newState;
            stateTimer.reset();
        }
    }


    private void stopAll() {
        //Robot.getInstance().launch.activeLaunch = false;
        Robot.getInstance().launchMotor.setPower(0.0);
        Robot.getInstance().intakeMotor.setPower(0.0);
        Robot.getInstance().transferServo.setPosition(SubsystemConstants.GOPOSITION);
//        Robot.getInstance().inServo.setPosition(SubsystemConstants.TRANSFER_REST);
    }


    @Override
    public void end() {
        stopAll();
    }
}

