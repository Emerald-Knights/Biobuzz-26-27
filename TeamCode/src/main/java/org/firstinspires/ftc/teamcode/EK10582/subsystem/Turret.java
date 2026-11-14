package org.firstinspires.ftc.teamcode.EK10582.subsystem;


import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.TrackingGoal.BLUEBACK;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.TrackingGoal.BLUEFRONT;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.TrackingGoal.REDBACK;
import static org.firstinspires.ftc.teamcode.EK10582.subsystem.SubsystemConstants.TrackingGoal.REDFRONT;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Turret extends Subsystem {

    protected Follower follower;
    private IMU imu;

    public static double P = 0.23, I = 0, D = 0, F = 12.85;
    public static double kP = 0.048001, kD = 0.0000001, kFF = 0.002;
    public double turretRotationInput;
    public boolean aimAssist;
    public boolean activeLaunch;
    public boolean manualOverride;
    public double turretSpinPower;
    boolean latePress = false;

    private double errorFilt = 0.0;
    private boolean errorFiltInit = false;
    private double lastErrorFilt = 0.0;
    private double derivFilt = 0.0;

    // Tune these
    public double alpha = 0.1;        // error filter (0.1–0.3)
    public double derivAlpha = 0.1;   // derivative filter (0.1–0.3)
    public double kS = 0.06;           // static friction feedforward (0.0–0.08)

    public boolean locked = false;
    public double lockInDeg = 0.8;   // enter lock
    public double lockOutDeg = 1.2;  // exit lock (must be > lockInDeg)
    public double lastPowerCmd = 0.0;
    private double lastYawDeg = 0.0;
    private double lastImuTime = 0.0;
    private double yawRateDegPerSec = 0.0;


    // ... other variables like kP, kD, etc.
    private double lastError = 0;

    public double targetLaunchVelocity;

    public Pose pose;

    public String invalid;

    public boolean detection;

    public double distance;

    public SubsystemConstants.TrackingGoal currentTargetGoal = SubsystemConstants.TrackingGoal.REDFRONT;

    public SubsystemConstants.TurretStates currentState = SubsystemConstants.TurretStates.MANUAL;

    public double xPose;
    public double yPose;
    public double botHeading;
    public double targetHeading;
    public double error;
    public double currentHeading;
    public double targetX;
    public double targetY;


    public double getError() {
        error = targetHeading - currentHeading;
        return error;
    }

    public void turretStop() {
        Robot.getInstance().launchMotor.setPower(0);
    }

    public void onNoTarget() {
        errorFiltInit = false;  // resets filter so we don't spike on reacquire
        derivFilt = 0;
        locked = false;
    }

    private double getYawDeg() {
        YawPitchRollAngles a = Robot.getInstance().imu.getRobotYawPitchRollAngles();
        return a.getYaw(AngleUnit.DEGREES);
    }

    private final double MAX_POWER = 0.5;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public void setkP(double newKP) {
        kP = newKP;
    }

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void turretUpdate(double currentHeading, double robotYawRateDegPerSec) {
        double deltaTime = timer.seconds();
        timer.reset();
        deltaTime = Math.max(deltaTime, 1e-3);

        if (currentTargetGoal == REDFRONT) {
            targetX = 60;
            targetY = 60;
        }

        if (currentTargetGoal == REDBACK) {
            targetX = 60;
            targetY = 84;
        }

        if (currentTargetGoal == BLUEFRONT) {
            targetX = 84;
            targetY = 60;
        }

        if (currentTargetGoal == BLUEBACK) {
            targetX = 84;
            targetY = 84;
        }

        targetHeading = Math.atan2(targetY - yPose, targetX - xPose);
        targetHeading = Math.toDegrees(targetHeading);
        double error = AngleUnit.normalizeDegrees(targetHeading - currentHeading);

        // --- Low-pass filter the error ---
        if (!errorFiltInit) {
            errorFilt = error;
            lastErrorFilt = errorFilt;
            derivFilt = 0.0;
            errorFiltInit = true;
        } else {
            errorFilt = (1 - alpha) * errorFilt + alpha * error;
        }


        // --- Deadband in degrees ---
        double absErr = Math.abs(errorFilt);


        if (locked) {
            if (absErr > lockOutDeg) locked = false;
        } else {
            if (absErr < lockInDeg) locked = true;
        }


        if (locked) {
            power = 0;
            Robot.getInstance().spinMotor.setPower(0);


            derivFilt = 0;           // kill D chatter
            lastError = error;
            lastErrorFilt = errorFilt;


            return;
        }
        //      Derivative using dt, and filter derivative ---
        double dErr = 0.0;
        if (deltaTime > 0) {
            dErr = (errorFilt - lastErrorFilt) / deltaTime;
        }
        derivFilt = (1 - derivAlpha) * derivFilt + derivAlpha * dErr;


        // --- PD output ---
        double output = (kP * errorFilt) + (kD * derivFilt);


        output += kFF * (-robotYawRateDegPerSec); // Feedforward: counteract robot rotation so tx doesn't "run away"


        // Optional static friction compensation (start with kS = 0)
        if (Math.abs(output) > 1e-6) {
            output += Math.signum(output) * kS;
        }


        if (Math.abs(output) < 0.005) output = 0;


        // Clamp
        power = Range.clip(output, -MAX_POWER, MAX_POWER);
        double maxDelta = 0.03; // tune 0.03–0.08
        power = Range.clip(power, lastPowerCmd - maxDelta, lastPowerCmd + maxDelta);
        lastPowerCmd = power;
        Robot.getInstance().spinMotor.setPower(power);


        // Update stored errors
        lastError = error;          // raw error for telemetry if you want
        lastErrorFilt = errorFilt;

    }

    public void init(boolean isAuton) {
        follower = Constants.create(Robot.getInstance().hardwareMap);
        pose = follower.pose();

        currentHeading = getYawDeg();
    }


    public void update(boolean isAuton) {
        follower.update();
        pose = follower.pose();
        xPose = pose.x();
        yPose = pose.y();
        currentHeading = Math.toDegrees(pose.heading());
        turretUpdate(currentHeading, getYawDeg());

        if(activeLaunch){
            Robot.getInstance().launchMotor.setVelocity(1000);
        }
        else {
            Robot.getInstance().launchMotor.setVelocity(0);
        }

    }

    public void stop() {
        Robot.getInstance().spinMotor.setPower(0);
        Robot.getInstance().launchMotor.setPower(0);

    }

    public void printToTelemetry(Telemetry telemetry) {
        telemetry.addData("X", xPose);
        telemetry.addData("Y", yPose);
        telemetry.addData("Turret Heading", currentHeading);
        telemetry.addData("Power", Robot.getInstance().launchMotor.getVelocity());


    }
}



