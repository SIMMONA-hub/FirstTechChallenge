package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Intake Resistance Test", group = "Test")
public class IntakeTest extends LinearOpMode {
    private DcMotorEx intakeMotor;

    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");

        telemetry.addLine("Ready. Press Start to measure intake resistance.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(-gamepad1.left_trigger);
            } else {
                intakeMotor.setPower(0);
            }

            double currentAmps = intakeMotor.getCurrent(CurrentUnit.AMPS);
            double velocity = intakeMotor.getVelocity();

            telemetry.addData("Power", "%.2f", intakeMotor.getPower());
            telemetry.addData("Current (AMPS)", "%.3f", currentAmps);
            telemetry.addData("Velocity (Ticks/sec)", "%.1f", velocity);
            
            telemetry.addLine();
            telemetry.addLine("1. Hold R-Trigger to spin intake");
            telemetry.addLine("2. Feed 3 balls to see how Amps & Velocity change");
            telemetry.addLine("3. When balls jam, Current will spike and Velocity will drop");
            telemetry.update();
        }
    }
}
