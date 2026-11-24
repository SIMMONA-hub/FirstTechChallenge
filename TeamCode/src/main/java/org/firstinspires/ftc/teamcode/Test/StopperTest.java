package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controllers.ShooterPIDVSA;
@TeleOp(name = "Stopper Test", group = "Test")
public class StopperTest extends LinearOpMode {

    private ShooterPIDVSA shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new ShooterPIDVSA();
        shooter.initialize(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                shooter.setStopper(ShooterPIDVSA.StopperPos.LOCKED);
            } else if (gamepad1.b) {
                shooter.setStopper(ShooterPIDVSA.StopperPos.RELEASED);
            }

            telemetry.addData("Stopper Lock Pos", ShooterPIDVSA.STOPPER_LOCK_POS);
            telemetry.addData("Stopper Release Pos", ShooterPIDVSA.STOPPER_RELEASE_POS);
            telemetry.update();
        }
    }
}