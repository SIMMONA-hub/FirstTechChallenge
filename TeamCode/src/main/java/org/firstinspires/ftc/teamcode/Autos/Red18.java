package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Controllers.Action;
import org.firstinspires.ftc.teamcode.Controllers.Turret;
import org.firstinspires.ftc.teamcode.Controllers.ShooterPIDVSA;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.BBQ.PoseStorage;

@Autonomous(name = "18Red", group = "Competition")
public class Red18 extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private ElapsedTime actionTimer;
    private int pathState = 0;
    private Action action;
    private Paths paths;

    public static class Poses {
        public static final Pose START = new Pose(115.215, 128.449, Math.toRadians(0));
    }

    public static class Paths {
        public PathChain StartToScore;
        public PathChain SCoreTOTake2;
        public PathChain Take2ToScore;
        public PathChain ScoreToGate;
        public PathChain GateTake;
        public PathChain GateToScore;
        public PathChain ScoreToTake1;
        public PathChain Take1ToScore;
        public PathChain ScoreToTake3;
        public PathChain TurnTake3;
        public PathChain Take3ToScore;

        public Paths(Follower follower) {
            StartToScore = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(115.215, 128.449),
                            new Pose(90.991, 89.383)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20)).build();

            SCoreTOTake2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(90.991, 89.383),
                            new Pose(93.818, 58.561),
                            new Pose(127.187, 58.991)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0)).build();

            Take2ToScore = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(127.187, 58.991),
                            new Pose(95.304, 69.355),
                            new Pose(90.991, 89.383)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            ScoreToGate = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(90.991, 89.383),
                            new Pose(104.047, 60.916),
                            new Pose(126.215, 61.131)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20)).build();

            GateTake = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(126.215, 61.131),
                            new Pose(126.121, 57.850)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(35)).build();

            GateToScore = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(126.121, 57.850),
                            new Pose(100.790, 68.696),
                            new Pose(90.991, 89.383)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0)).build();

            ScoreToTake1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(90.991, 89.383),
                            new Pose(109.458, 82.093),
                            new Pose(126.467, 83.430)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Take1ToScore = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(126.467, 83.430),
                            new Pose(90.991, 89.383)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            ScoreToTake3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(90.991, 89.383),
                            new Pose(82.458, 30.364),
                            new Pose(126.336, 34.766)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            TurnTake3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(126.336, 34.766),
                            new Pose(127.336, 34.766)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-67)).build();

            Take3ToScore = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(127.336, 34.766),
                            new Pose(92.542, 117.748)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(-67), Math.toRadians(-67)).build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                action.setTurretMode(Turret.Mode.ODOMETRY);
                action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                follower.followPath(paths.StartToScore);
                follower.setMaxPower(1);
                action.servoMidle();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    action.Sequence();
                    actionTimer.reset();
                    setPathState(2);
                }
                break;
            case 2:
                if (actionTimer.milliseconds() > 700) {
                    action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                    follower.followPath(paths.SCoreTOTake2);
                    action.take();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Take2ToScore);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    action.Sequence();
                    actionTimer.reset();
                    setPathState(5);
                }
                break;
            case 5:
                if (actionTimer.milliseconds() > 700) {
                    action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                    follower.followPath(paths.ScoreToGate);
                    action.stopTake();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateTake);
                    action.take();
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateToScore);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    action.Sequence();
                    actionTimer.reset();
                    setPathState(9);
                }
                break;
            case 9:
                if (actionTimer.milliseconds() > 600) {
                    action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                    follower.followPath(paths.ScoreToTake1);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Take1ToScore);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    action.Sequence();
                    actionTimer.reset();
                    setPathState(12);
                }
                break;
            case 12:
                if (actionTimer.milliseconds() > 700) {
                    action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                    follower.followPath(paths.ScoreToGate);
                    action.stopTake();
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateTake);
                    action.take();
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateToScore);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    action.Sequence();
                    actionTimer.reset();
                    setPathState(16);
                }
                break;
            case 16:
                if (actionTimer.milliseconds() > 600) {
                    action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                    follower.followPath(paths.ScoreToTake3);
                    action.take();
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(paths.TurnTake3);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Take3ToScore);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    action.Sequence();
                    actionTimer.reset();
                    setPathState(20);
                }
                break;
            case 20:
                if (actionTimer.milliseconds() > 700) {
                    action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                    action.stopTake();
                    action.stopShooter();
                    setPathState(21);
                }
                break;
            case 21:
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.START);
        paths = new Paths(follower);
        action = new Action(hardwareMap);
        Action.isBlue = false;
        action.runShooterRaw();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        action.runCloseShooter();
        autonomousPathUpdate();
        action.updateAll(follower.getPose());
        PoseStorage.currentPose = follower.getPose();
        PoseStorage.lastTurretAngle = action.getCurrentTurretAngle();
        telemetry.addData("State", pathState);
        telemetry.addData("Turret Angle", action.getTurretAngle());
        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
        PoseStorage.lastTurretAngle = action.getCurrentTurretAngle();
        PoseStorage.usePoseStorage = true;
    }
}
