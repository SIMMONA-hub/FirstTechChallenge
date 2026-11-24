package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.BBQ.ShooterMath;

@Config
public class Action {
    private Intake intake;
    private ShooterPIDVSA shooter;
    private Turret turret;

    public static double TARGET_X = 144;
    public static double TARGET_Y = 144;
    public static double TARGET_BLUE_X = 0;
    public static double TARGET_BLUE_Y = 144;

    public static double SHOOTER_TARGET_X = 122;
    public static double SHOOTER_TARGET_Y = 115;
    public static double SHOOTER_TARGET_BLUE_X = 22;
    public static double SHOOTER_TARGET_BLUE_Y = 115;

    public static double RPM_CLOSE = 1150;
    public static double RPM_FAR = 1600;
    public static double RedFar15 = 60;
    public static double RedFar152 = 37;
    public static double RedClose = 80;
    public static double BlueClose = 90;
    public static double BlueFar15 = -60;
    public static double BlueFar152 = -37;

    private boolean shooterEnabled = false;
    private boolean autoShootEnabled = false;
    private boolean autoModeActive = false;
    public static boolean isBlue = false;

    public Action(HardwareMap hardwareMap, boolean resetTurret) {
        intake = new Intake();
        shooter = new ShooterPIDVSA();
        turret = new Turret();

        intake.initialize(hardwareMap);
        shooter.initialize(hardwareMap);
        turret.init(hardwareMap, resetTurret);
    }

    public Action(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    public void updateAll(Pose robotPose) {
        if (shooterEnabled) {
            if (autoShootEnabled) {
                double curShooterTargetX = isBlue ? SHOOTER_TARGET_BLUE_X : SHOOTER_TARGET_X;
                double curShooterTargetY = isBlue ? SHOOTER_TARGET_BLUE_Y : SHOOTER_TARGET_Y;
                double dist = Math.hypot(curShooterTargetX - robotPose.getX(), curShooterTargetY - robotPose.getY());

                shooter.setTargetRPM(ShooterMath.flywheelRPM(dist));
                shooter.setHoodPosition(ShooterMath.hoodPos(dist));
            }
            shooter.update();
        } else {
            shooter.stopShooter();
        }

        turret.update(robotPose);
    }

    public void Sequence() {
        shooter.setStopper(ShooterPIDVSA.StopperPos.RELEASED);
        intake.setIntakePower(1);
    }
    public void FarSequence() {
        shooter.setStopper(ShooterPIDVSA.StopperPos.RELEASED);
        intake.setIntakePower(0.75);
    }

    public void take() {
        shooter.setStopper(ShooterPIDVSA.StopperPos.LOCKED);
        intake.setIntakePower(1);
    }

    public void setStopperState(ShooterPIDVSA.StopperPos pos) {
        shooter.setStopper(pos);
    }

    public void stopTake() {
        intake.setIntakePower(0);
        shooter.setStopper(ShooterPIDVSA.StopperPos.LOCKED);
    }
    public void Locked(){
        shooter.setStopper(ShooterPIDVSA.StopperPos.LOCKED);
    }

    public boolean isIntakeFull() {
        return intake.isFull();
    }

    public void reverseIntake() {
        intake.setIntakePower(-1);
        shooter.setStopper(ShooterPIDVSA.StopperPos.LOCKED);
    }

    public static Pose getActiveTarget() {
        return isBlue ? new Pose(TARGET_BLUE_X, TARGET_BLUE_Y) : new Pose(TARGET_X, TARGET_Y);
    }

    public double getCurrentDistance(Pose robotPose) {
        double curShooterTargetX = isBlue ? SHOOTER_TARGET_BLUE_X : SHOOTER_TARGET_X;
        double curShooterTargetY = isBlue ? SHOOTER_TARGET_BLUE_Y : SHOOTER_TARGET_Y;
        return Math.hypot(curShooterTargetX - robotPose.getX(), curShooterTargetY - robotPose.getY());
    }

    public void startAutoShooter() {
        this.autoModeActive = true;
        this.shooterEnabled = true;
        this.autoShootEnabled = true;
    }

    public double getShooterTargetRPM() {
        return shooter.getTargetRPM();
    }

    public void runCloseShooter(){
        this.shooterEnabled = true;
        shooter.setTargetRPM(1200);
    }

    public void runFarShoot(){
        this.shooterEnabled = true;
        shooter.setTargetRPM(1800);
    }

    public void triggerShooter() {
        shooterEnabled = true;
        autoShootEnabled = autoModeActive;
        if (!autoModeActive) shooter.setTargetRPM(RPM_CLOSE);
    }

    public void stopShooter() {
        shooterEnabled = false;
        autoShootEnabled = false;
        shooter.stopShooter();
    }

    public double getShooterCurrentRPM() {
        return shooter.getCurrentRPM();
    }

    public void setTurretMode(Turret.Mode mode) {
        turret.setMode(mode);
    }

    public double getTurretAngle() {
        return turret.getTargetAngle();
    }

    public double getCurrentTurretAngle() {
        return turret.getCurrentAngleDeg();
    }

    public void centerTurret() {
        turret.setMode(Turret.Mode.MANUAL);
        turret.setManualAngle(0);
    }

    public void toggleAutoMode() {
        autoModeActive = !autoModeActive;
        if (shooterEnabled) {
            autoShootEnabled = autoModeActive;
            if (!autoModeActive) {
                shooter.setTargetRPM(RPM_CLOSE);
            }
        }
    }

    public void setAutoMode(boolean active) {
        this.autoModeActive = active;
    }

    public boolean isAutoModeActive() {
        return autoModeActive;
    }

    public void runFarShooter() {
        autoShootEnabled = false;
        shooter.setTargetRPM(RPM_FAR);
    }

    public void runShooterRaw() {
        shooterEnabled = true;
    }

    public void servoUp() { shooter.setServo(ShooterPIDVSA.ServosPos.UP); }
    public void servoMidle() { shooter.setServo(ShooterPIDVSA.ServosPos.MIDDLE); }
    public void servoDown() { shooter.setServo(ShooterPIDVSA.ServosPos.DOWN); }

    public void FarRed(){
        turret.setDegreeLock(RedFar15);
    }
    public void FarRed2(){
        turret.setDegreeLock(RedFar152);
    }
    public void FarBlue(){
        turret.setDegreeLock(BlueFar15);
    }
    public void FarBlue2(){
        turret.setDegreeLock(BlueFar152);
    }

    public void CloseRed(){
        turret.setDegreeLock(RedClose);
    }
    public void CLoseBlue(){
        turret.setDegreeLock(BlueClose);
    }
}