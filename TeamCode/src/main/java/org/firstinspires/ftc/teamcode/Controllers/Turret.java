package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Turret {

    public enum Mode {
        ODOMETRY,
        FIELD_45,
        FIELD_NEG_45,
        DEGREE_LOCK,
        MOVING_SHOT,
        MANUAL,
        DISABLED
    }

    private Mode currentMode = Mode.DISABLED;
    private DcMotor turretMotor;

    public static double TICKS_PER_DEGREE = 4.533;
    public static double MIN_ANGLE = -135;
    public static double MAX_ANGLE =  200;

    public static double kP            = 0.1;
    public static double kI            = 0.0;
    public static double kD            = 0.00003;
    public static double MAX_POWER     = 1;
    public static double TOLERANCE_DEG = 1.5;

    public static double PROJECTILE_SPEED = 6.0;
    public static double MIN_FLIGHT_TIME  = 0.05;
    public static double MAX_FLIGHT_TIME  = 0.4;
    public static int    PREDICTION_ITERS = 3;

    private double targetAngle      = 0;
    private double lockedFieldAngle = 0;

    private double integralSum = 0;
    private double lastError   = 0;
    private double lastPidTime = 0;
    private int    zeroPoseTicks = 0;

    private Pose   lastPose     = null;
    private double lastPoseTime = 0;
    private double velX = 0, velY = 0, velH = 0;

    private final ElapsedTime clock = new ElapsedTime();

    public void init(HardwareMap hardwareMap, boolean resetEncoder) {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        if (resetEncoder) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        zeroPoseTicks = 0;

        clock.reset();
        lastPidTime = clock.seconds();
    }

    public void resetPosition() {
        zeroPoseTicks = turretMotor.getCurrentPosition();
        integralSum   = 0;
        lastError     = 0;
        targetAngle   = 0;
        lastPose      = null;
    }

    public void update(Pose robotPose) {
        estimateVelocities(robotPose);

        if (currentMode == Mode.DISABLED) {
            turretMotor.setPower(0);
            return;
        }

        switch (currentMode) {
            case ODOMETRY: {
                Pose target = Action.getActiveTarget();
                targetAngle = calculateOdoAngle(robotPose, target.getX(), target.getY());
                break;
            }

            case MOVING_SHOT: {
                Pose movingTarget = Action.getActiveTarget();
                targetAngle = calculateMovingShotAngle(robotPose, movingTarget.getX(), movingTarget.getY());
                break;
            }

            case FIELD_45:
                targetAngle = calculateFixedFieldAngle(robotPose, 45);
                break;

            case FIELD_NEG_45:
                targetAngle = calculateFixedFieldAngle(robotPose, 134);
                break;

            case DEGREE_LOCK:
                targetAngle = calculateFixedFieldAngle(robotPose, lockedFieldAngle);
                break;
        }

        targetAngle = wrapToLimits(targetAngle);
        targetAngle = clamp(targetAngle, MIN_ANGLE, MAX_ANGLE);

        driveToAngle(targetAngle);
    }

    private void driveToAngle(double targetAng) {
        double now = clock.seconds();
        double dt  = Math.max(now - lastPidTime, 1e-4);
        lastPidTime = now;

        double currentAngle = getCurrentAngleDeg();
        double error = targetAng - currentAngle;

        if (Math.abs(error) < TOLERANCE_DEG) {
            turretMotor.setPower(0);
            integralSum = 0;
            lastError   = error;
            return;
        }

        integralSum += error * dt;
        integralSum = clamp(integralSum, -0.2 / (kI + 1e-6), 0.2 / (kI + 1e-6));

        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (kP * error) + (kI * integralSum) + (kD * derivative);
        turretMotor.setPower(clamp(power, -MAX_POWER, MAX_POWER));
    }

    private void estimateVelocities(Pose robotPose) {
        double now = clock.seconds();
        if (lastPose != null) {
            double dt = now - lastPoseTime;
            if (dt > 0.005) {
                double rawVelX = (robotPose.getX()       - lastPose.getX())       / dt;
                double rawVelY = (robotPose.getY()       - lastPose.getY())       / dt;
                double rawVelH = (robotPose.getHeading() - lastPose.getHeading()) / dt;

                double alpha = 0.3;
                velX = alpha * rawVelX + (1 - alpha) * velX;
                velY = alpha * rawVelY + (1 - alpha) * velY;
                velH = alpha * rawVelH + (1 - alpha) * velH;

                lastPose     = robotPose;
                lastPoseTime = now;
            }
        } else {
            lastPose     = robotPose;
            lastPoseTime = now;
        }
    }

    private double calculateMovingShotAngle(Pose robotPose, double tx, double ty) {
        double flightTime = 0.1;
        double predictedRobotX       = robotPose.getX();
        double predictedRobotY       = robotPose.getY();
        double predictedRobotHeading = robotPose.getHeading();

        for (int i = 0; i < PREDICTION_ITERS; i++) {
            predictedRobotX       = robotPose.getX()       + velX * flightTime;
            predictedRobotY       = robotPose.getY()       + velY * flightTime;
            predictedRobotHeading = robotPose.getHeading() + velH * flightTime;

            double dx       = tx - predictedRobotX;
            double dy       = ty - predictedRobotY;
            double distance = Math.sqrt(dx * dx + dy * dy);

            flightTime = distance / PROJECTILE_SPEED;
            flightTime = clamp(flightTime, MIN_FLIGHT_TIME, MAX_FLIGHT_TIME);
        }

        double dx            = tx - predictedRobotX;
        double dy            = ty - predictedRobotY;
        double absoluteAngle = Math.toDegrees(Math.atan2(dy, dx));
        double relativeAngle = absoluteAngle - Math.toDegrees(predictedRobotHeading);

        return relativeAngle;
    }

    private double calculateOdoAngle(Pose robotPose, double tx, double ty) {
        double dx = tx - robotPose.getX();
        double dy = ty - robotPose.getY();
        double absoluteAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading  = Math.toDegrees(robotPose.getHeading());
        return absoluteAngle - robotHeading;
    }

    private double calculateFixedFieldAngle(Pose robotPose, double fieldAngle) {
        return fieldAngle - Math.toDegrees(robotPose.getHeading());
    }

    public void setMode(Mode mode) {
        if (currentMode != mode) {
            integralSum = 0;
            lastError   = 0;
        }
        currentMode = mode;
    }

    public void setDegreeLock(double fieldAngleDeg) {
        this.lockedFieldAngle = fieldAngleDeg;
        setMode(Mode.DEGREE_LOCK);
    }

    public void setManualAngle(double angleDeg) {
        if (currentMode != Mode.MANUAL) return;
        targetAngle = clamp(angleDeg, MIN_ANGLE, MAX_ANGLE);
        driveToAngle(targetAngle);
    }

    public double getCurrentAngleDeg() {
        return (turretMotor.getCurrentPosition() - zeroPoseTicks) / TICKS_PER_DEGREE;
    }

    public int getCurrentTicks()   { return turretMotor.getCurrentPosition(); }
    public Mode getMode()          { return currentMode; }
    public double getTargetAngle() { return targetAngle; }
    public double getVelX()        { return velX; }
    public double getVelY()        { return velY; }
    public double getVelH()        { return Math.toDegrees(velH); }

    public boolean isOnTarget() {
        if (currentMode == Mode.DISABLED || currentMode == Mode.MANUAL) return false;
        return Math.abs(targetAngle - getCurrentAngleDeg()) < TOLERANCE_DEG;
    }

    private double wrapToLimits(double angle) {
        double normalized = angle;
        while (normalized > 180) normalized -= 360;
        while (normalized < -180) normalized += 360;

        if (normalized < MIN_ANGLE && (normalized + 360) <= MAX_ANGLE) {
            normalized += 360;
        } else if (normalized > MAX_ANGLE && (normalized - 360) >= MIN_ANGLE) {
            normalized -= 360;
        }
        return normalized;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}