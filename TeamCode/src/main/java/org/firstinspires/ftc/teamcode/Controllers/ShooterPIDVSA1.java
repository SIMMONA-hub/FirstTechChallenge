package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.*;

@Config
public class ShooterPIDVSA1 {
    private DcMotorEx ShooterLeft;
    private DcMotorEx ShooterRight;
    private Servo ServoHood;
    private Servo ServoStopper;
    private VoltageSensor batteryVoltageSensor;

    public static double kS = 0.1;
    public static double kV = 0.0039;
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double TARGET_RPM = 0;
    public static double MAX_RPM = 6000;

    public static double SERVO_DOWN_POS = 1;
    public static double SERVO_MIDDLE_POS = 0.5;
    public static double SERVO_UP_POS = 0.22;
    public static double STOPPER_LOCK_POS = 0.1;
    public static double STOPPER_RELEASE_POS = 0.35;

    private double lastError = 0;
    private double integral = 0;
    private long lastTime = 0;

    public enum ServosPos {
        DOWN,
        MIDDLE,
        UP
    }

    public enum StopperPos {
        LOCKED,
        RELEASED
    }

    public void initialize(HardwareMap hardwareMap) {
        ShooterLeft = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        ShooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");
        ServoHood = hardwareMap.get(Servo.class, "LeftKozel");
        ServoStopper = hardwareMap.get(Servo.class, "Stopper");

        ShooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ServoHood.setDirection(Servo.Direction.FORWARD);
        ServoStopper.setDirection(Servo.Direction.FORWARD);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        lastTime = System.nanoTime();

        setServo(ServosPos.DOWN);
        setStopper(StopperPos.LOCKED);
    }

    public void update() {
        if (TARGET_RPM <= 0) {
            stopShooter();
            return;
        }

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) * 1e-9;
        if (dt < 0.001) return;
        if (dt > 0.1) dt = 0.02;
        lastTime = currentTime;

        double currentVelocity = ShooterLeft.getVelocity();

        double feedforwardVoltage = kS * Math.signum(TARGET_RPM) + kV * TARGET_RPM;

        double error = TARGET_RPM - currentVelocity;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidVoltage = (kP * error) + (kI * integral) + (kD * derivative);
        double totalVoltage = feedforwardVoltage + pidVoltage;

        double currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        if (currentBatteryVoltage <= 0) currentBatteryVoltage = 12.0;

        double power = totalVoltage / currentBatteryVoltage;
        power = clamp(power, 0.0, 1.0);

        ShooterLeft.setPower(power);
        ShooterRight.setPower(power);
    }

    public void stopShooter() {
        TARGET_RPM = 0;
        integral = 0;
        lastError = 0;
        lastTime = System.nanoTime();
        ShooterLeft.setPower(0);
        ShooterRight.setPower(0);
    }

    public void setTargetRPM(double rpm) {
        TARGET_RPM = clamp(rpm, 0, MAX_RPM);
    }

    public double getTargetRPM() {
        return TARGET_RPM;
    }

    public boolean isAtSpeed(double tolerance) {
        return Math.abs(ShooterLeft.getVelocity() - TARGET_RPM) < tolerance;
    }

    public void setHoodPosition(double pos) {
        double min = Math.min(SERVO_DOWN_POS, SERVO_UP_POS);
        double max = Math.max(SERVO_DOWN_POS, SERVO_UP_POS);
        pos = clamp(pos, min, max);
        ServoHood.setPosition(pos);
    }

    public void setServo(ServosPos pos) {
        double p = pos == ServosPos.DOWN ? SERVO_DOWN_POS :
                pos == ServosPos.MIDDLE ? SERVO_MIDDLE_POS :
                        SERVO_UP_POS;
        p = clamp(p, 0.0, 1.0);
        ServoHood.setPosition(p);
    }

    public void setStopper(StopperPos pos) {
        double p = pos == StopperPos.LOCKED ? STOPPER_LOCK_POS : STOPPER_RELEASE_POS;
        p = clamp(p, 0.0, 1.0);
        ServoStopper.setPosition(p);
    }

    public double getCurrentRPM() {
        return ShooterLeft.getVelocity();
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }
}