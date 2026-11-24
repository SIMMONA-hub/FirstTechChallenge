//package org.firstinspires.ftc.teamcode.Test;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.BBQ.ELAgamepadex;
//import org.firstinspires.ftc.teamcode.BBQ.PoseStorage;
//import org.firstinspires.ftc.teamcode.Controllers.Base;
//import org.firstinspires.ftc.teamcode.Controllers.Turret;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@TeleOp(name = "TurretTest", group = "Test")
//public class TurretTest extends LinearOpMode {
//    private Turret turret;
//    private Base driveBase;
//    private Follower follower;
//    private ELAgamepadex driver1;
//
//    @Override
//    public void runOpMode() {
//        follower = Constants.createFollower(hardwareMap);
//
//        Pose startPose = PoseStorage.currentPose;
//        follower.setStartingPose(startPose);
//        follower.setPose(startPose);
//
//        driver1 = new ELAgamepadex(gamepad1);
//
//        turret = new Turret();
//        turret.init(hardwareMap);
//
//        driveBase = new Base();
//        driveBase.initialize(hardwareMap, true);
//
//        while (!isStarted() && !isStopRequested()) {
//            telemetry.addLine("=== TURRET TEST ===");
//            telemetry.addData("Turret Angle", "%.1f°", turret.getCurrentAngleDeg());
//            telemetry.addData("Turret Ticks", turret.getCurrentTicks());
//            telemetry.addLine();
//            telemetry.addLine("B = Reset Pose + Heading");
//            telemetry.addLine("X = Odometry Mode");
//            telemetry.addLine("DPAD_UP = Moving Shot");
//            telemetry.addLine("DPAD_LEFT = Disabled");
//            telemetry.addLine("A = Reset Turret Encoder");
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            driver1.update();
//            follower.update();
//
//            Pose currentPose = follower.getPose();
//
//            driveBase.update(
//                    driver1.getLeftStickX(),
//                    -driver1.getLeftStickY(),
//                    driver1.getRightStickX(),
//                    1.0,
//                    false,
//                    false
//            );
//
//            // X — включить режим одометрии
//            if (driver1.isXPressed()) {
//                turret.setMode(Turret.Mode.ODOMETRY);
//            }
//
//            // DPAD_UP — Moving Shot
//            if (driver1.isDpadUpPressed()) {
//                turret.setMode(Turret.Mode.MOVING_SHOT);
//            }
//
//            // DPAD_LEFT — выключить турель
//            if (driver1.isDpadLeftPressed()) {
//                turret.setMode(Turret.Mode.DISABLED);
//            }
//
//            // A — ресетнуть энкодер турельки (текущая позиция = 0°)
//            if (driver1.isAPressed()) {
//                turret.resetPosition();
//            }
//
//            // B — ресет позы и хединга
//            if (driver1.isBPressed()) {
//                Pose resetP = new Pose(125.7, 74.7, 0);
//                follower.setPose(resetP);
//                driveBase.resetHeading();
//            }
//
//            turret.update(currentPose);
//
//            // Телеметрия
//            telemetry.addData("MODE", turret.getMode());
//            telemetry.addData("Turret Angle", "%.1f°", turret.getCurrentAngleDeg());
//            telemetry.addData("Target Angle", "%.1f°", turret.getTargetAngle());
//            telemetry.addData("On Target", turret.isOnTarget());
//            telemetry.addData("Ticks", turret.getCurrentTicks());
//            telemetry.addData("VelX", "%.2f", turret.getVelX());
//            telemetry.addData("VelY", "%.2f", turret.getVelY());
//            telemetry.addData("VelH", "%.1f°/s", turret.getVelH());
//            telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.1f°",
//                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
//            telemetry.update();
//        }
//    }
//}
