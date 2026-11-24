package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake {
    private DcMotorEx Intake;
    private ElapsedTime jamTimer = new ElapsedTime();

    public void initialize(HardwareMap hardwareMap){
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        jamTimer.reset();
    }

    public void setIntakePower(double power){
        double limitedPower = Math.max(-1.0, Math.min(power, 1.0));
        Intake.setPower(limitedPower);
    }

    public boolean isFull() {
        if (Intake.getCurrent(CurrentUnit.AMPS) > 4.2) {
            return jamTimer.milliseconds() > 500;
        } else {
            jamTimer.reset();
            return false;
        }
    }
}