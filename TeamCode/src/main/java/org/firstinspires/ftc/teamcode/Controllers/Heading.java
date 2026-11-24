package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Heading {
    private GoBildaPinpointDriver pinpoint;
    private double currentHeadingRad = 0;
    private double previousRawHeading = 0;
    private double velX = 0;
    private double velY = 0;
    private double headingVelocity = 0;
    private int nanCounter = 0;
    private static final int MAX_NAN_COUNT = 5;

    public Heading(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        initializePinpoint(-2.95276, 0);
    }

    private void initializePinpoint(double offsetX, double offsetY) {
        pinpoint.initialize();
        pinpoint.resetDeviceConfigurationForOpMode();
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
        pinpoint.setOffsets(offsetX, offsetY, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        double initialHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        if (Double.isFinite(initialHeading)) {
            currentHeadingRad = initialHeading;
            previousRawHeading = initialHeading;
        }
    }

    public void update() {
        pinpoint.update();
        velX = pinpoint.getVelX(DistanceUnit.INCH);
        velY = pinpoint.getVelY(DistanceUnit.INCH);
        headingVelocity = pinpoint.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized());

        double rawHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        double deltaHeading = 0;

        if (!Double.isFinite(rawHeading)) {
            nanCounter++;
            if (nanCounter > MAX_NAN_COUNT) return;
            rawHeading = previousRawHeading;
        } else {
            nanCounter = 0;
            deltaHeading = rawHeading - previousRawHeading;
            if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
            else if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;
            previousRawHeading = rawHeading;
        }

        currentHeadingRad += deltaHeading;
        currentHeadingRad = normalizeAngle(currentHeadingRad);
    }

    private double normalizeAngle(double angle) {
        angle %= (2 * Math.PI);
        if (angle > Math.PI) angle -= 2 * Math.PI;
        else if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void setPose(Pose newPose) {
        pinpoint.setPosX(newPose.getX(), DistanceUnit.INCH);
        pinpoint.setPosY(newPose.getY(), DistanceUnit.INCH);
        pinpoint.setHeading(newPose.getHeading(), AngleUnit.RADIANS);
    }

    public double getVelX() { return velX; }
    public double getVelY() { return velY; }
    public double getHeadingVelocity() { return headingVelocity; }
    public double getCurrentHeading() { return currentHeadingRad; }

    public void resetPinpoint() {
        currentHeadingRad = 0;
        previousRawHeading = 0;
        pinpoint.resetPosAndIMU();
    }
}