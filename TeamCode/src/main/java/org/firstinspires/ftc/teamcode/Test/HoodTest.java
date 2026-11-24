package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controllers.ShooterPIDVSA;

@TeleOp(name = "Hood Test", group = "Test")
public class HoodTest extends LinearOpMode {

    private ShooterPIDVSA shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new ShooterPIDVSA();
        shooter.initialize(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                shooter.setServo(ShooterPIDVSA.ServosPos.DOWN);
            } else if (gamepad1.y) {
                shooter.setServo(ShooterPIDVSA.ServosPos.MIDDLE);
            } else if (gamepad1.b) {
                shooter.setServo(ShooterPIDVSA.ServosPos.UP);
            }

            telemetry.addData("Down Pos", ShooterPIDVSA.SERVO_DOWN_POS);
            telemetry.addData("Middle Pos", ShooterPIDVSA.SERVO_MIDDLE_POS);
            telemetry.addData("Up Pos", ShooterPIDVSA.SERVO_UP_POS);
            telemetry.update();
        }
    }
}