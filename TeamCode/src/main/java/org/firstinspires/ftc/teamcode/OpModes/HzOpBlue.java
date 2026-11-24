package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.BBQ.ELAgamepadex;
import org.firstinspires.ftc.teamcode.BBQ.PoseStorage;
import org.firstinspires.ftc.teamcode.Controllers.Action;
import org.firstinspires.ftc.teamcode.Controllers.Base;
import org.firstinspires.ftc.teamcode.Controllers.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "HzOpBlue", group = "Competition")
public class HzOpBlue extends LinearOpMode {
    private Action action;
    private Base driveBase;
    private Follower follower;
    private ELAgamepadex driver1;
    private boolean wasIntakeFull = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);

        Action.isBlue = true;
        Pose startPose = PoseStorage.usePoseStorage ? PoseStorage.currentPose : new Pose(27.664, 127.776, Math.toRadians(180));
        PoseStorage.usePoseStorage = false;
        follower.setStartingPose(startPose);
        follower.setPose(startPose);

        driver1 = new ELAgamepadex(gamepad1);

        action = new Action(hardwareMap, false);

        driveBase = new Base();
        driveBase.initialize(hardwareMap, true);

        Base.fieldCentricOffset = Math.toRadians(180);
        driveBase.resetHeading(Math.toRadians(180));

        action.setAutoMode(true);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("READY - BLUE SIDE HZ OP");
            telemetry.addData("DEFAULT MODE", "AUTO RPM (shooter off)");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            driver1.update();
            follower.update();

            boolean isIntakeFull = action.isIntakeFull();
            if (isIntakeFull && !wasIntakeFull) {
                gamepad1.rumble(500);
            }
            wasIntakeFull = isIntakeFull;

            Pose currentPose = follower.getPose();
            action.updateAll(currentPose);

            driveBase.update(
                    driver1.getLeftStickX(),
                    -driver1.getLeftStickY(),
                    driver1.getRightStickX(),
                    1.0,
                    false,
                    false
            );

            if (driver1.isRightTriggerDown()) {
                action.take();
            } else if (driver1.isLeftTriggerDown()) {
                if (action.getCurrentDistance(currentPose) > 75) {
                    action.FarSequence();
                } else {
                    action.Sequence();
                }
            } else if (driver1.isDpadDownDown()) {
                action.reverseIntake();
            } else {
                action.stopTake();
            }

            if (driver1.isRightBumperPressed()) {
                action.triggerShooter();
            } else if (driver1.isLeftBumperPressed()) {
                action.stopShooter();
            }

            if (driver1.isAPressed()) {
                action.servoMidle();
            }

            if (driver1.isXPressed()) {
                action.setTurretMode(Turret.Mode.ODOMETRY);
            } else if (driver1.isDpadUpPressed()) {
                action.setTurretMode(Turret.Mode.MOVING_SHOT);
            } else if (driver1.isDpadLeftPressed()) {
                action.centerTurret();
            }

            if (driver1.isYPressed()) {
                action.servoUp();
                action.runFarShooter();
                action.runShooterRaw();
            }

            if (driver1.isDpadRightPressed()) {
                action.toggleAutoMode();
            }

            if (driver1.isBPressed()) {
                Pose resetP = new Pose(16.523, 78.878, Math.toRadians(180));
                follower.setPose(resetP);
                driveBase.resetHeading(Math.toRadians(180));
            }

            printTelemetry(currentPose);
        }
    }

    private void printTelemetry(Pose pose) {
        telemetry.addData("ALLIANCE", Action.isBlue ? "BLUE" : "RED");
        telemetry.addData("SHOOTER RPM MODE", action.isAutoModeActive() ? "AUTO (DISTANCE MATH)" : "MANUAL");
        telemetry.addData("Target RPM", "%.0f", action.getShooterTargetRPM());
        telemetry.addData("Current RPM", "%.0f", action.getShooterCurrentRPM());
        telemetry.addData("Turret Angle", "%.1f", action.getTurretAngle());
        telemetry.addData("Distance", "%.2f", action.getCurrentDistance(pose));
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}