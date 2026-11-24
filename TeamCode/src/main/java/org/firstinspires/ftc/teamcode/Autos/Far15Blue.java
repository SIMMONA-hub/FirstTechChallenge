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

@Autonomous(name = "FarBlue", group = "Competition")
public class Far15Blue extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private ElapsedTime actionTimer;
    private int pathState = 0;
    private Action action;
    private Paths paths;

    public static class Poses {
        public static final Pose START = new Pose(65.121, 8.673, Math.toRadians(180));
    }

    public static class Paths {
        public PathChain StartToScore;
        public PathChain ScoreToTake3;
        public PathChain TurnAtTake3;
        public PathChain Take3ToScore;
        public PathChain ScoreToHumanTake;
        public PathChain HumanTakeToScore;
        public PathChain ScoreToGateTake;
        public PathChain GateTakeToScore;
        public PathChain Leave;

        public Paths(Follower follower) {
            StartToScore = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(65.121, 8.673), new Pose(62.654, 17.607))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120)).build();


            TurnAtTake3 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(17.738, 35.196), new Pose(17.738, 35.196))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157)).build();

            Take3ToScore = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(17.738, 35.196), new Pose(62.654, 17.607))
            ).setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(157)).build();

            ScoreToHumanTake = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(62.654, 17.607), new Pose(34.715, 7.453), new Pose(9.486, 7.561))
            ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180)).build();

            HumanTakeToScore = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(9.486, 7.561), new Pose(62.654, 17.607))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120)).build();

            ScoreToGateTake = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(62.654, 17.607), new Pose(11.131, 13.397), new Pose(10.916, 35.280))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130)).build();

            GateTakeToScore = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(10.916, 35.280), new Pose(62.654, 17.607))
            ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180)).build();

            Leave = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(62.654, 17.607), new Pose(62.299, 32.374))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                action.setTurretMode(Turret.Mode.ODOMETRY);
                action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                follower.followPath(paths.StartToScore);
                follower.setMaxPower(1);
                action.servoUp();
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
                if (actionTimer.milliseconds() > 800) {
                    action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                    follower.followPath(paths.ScoreToHumanTake);
                    action.take();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    action.stopTake();
                    follower.followPath(paths.HumanTakeToScore);
                    setPathState(51);
                }
                break;
            case 51:
                if (!follower.isBusy()) {
                    action.Sequence();
                    actionTimer.reset();
                    setPathState(6);
                }
                break;
            case 6:
                if (actionTimer.milliseconds() > 800) {
                    action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                    follower.followPath(paths.ScoreToHumanTake);
                    action.take();
                    setPathState(61);
                }
                break;
            case 61:
                if (!follower.isBusy()) {
                    action.stopTake();
                    follower.followPath(paths.HumanTakeToScore);
                    setPathState(71);
                }
                break;
            case 71:
                if (!follower.isBusy()) {
                    action.Sequence();
                    actionTimer.reset();
                    setPathState(11);
                }
                break;
            case 11:
                if (actionTimer.milliseconds() > 800) {
                    action.setStopperState(ShooterPIDVSA.StopperPos.LOCKED);
                    follower.followPath(paths.ScoreToHumanTake);
                    action.take();
                    setPathState(112);
                }
                break;
            case 112:
                if (!follower.isBusy()) {
                    action.stopTake();
                    follower.followPath(paths.HumanTakeToScore);
                    setPathState(171);
                }
                break;
            case 171:
                if (!follower.isBusy()) {
                    action.Sequence();
                    actionTimer.reset();
                    setPathState(181);
                }
                break;
            case 181:
                if (actionTimer.milliseconds() > 800) {
                    action.stopTake();
                    follower.followPath(paths.Leave);
                    setPathState(111);
                }
                break;
            case 111:
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
        action.runFarShoot();
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
        telemetry.addData("State", pathState);
        telemetry.addData("Turret Angle", action.getTurretAngle());
        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}