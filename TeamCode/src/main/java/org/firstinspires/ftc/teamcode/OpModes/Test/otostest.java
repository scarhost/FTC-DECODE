package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "otostest")
public class otostest extends LinearOpMode {
    SparkFunOTOS otos;
    @Override
    public void runOpMode() throws InterruptedException {
        this.otos=hardwareMap.get(SparkFunOTOS.class, "otos");
        initializeOtos();
        waitForStart();
        while (opModeIsActive()){
            SparkFunOTOS.Pose2D position = this.otos.getPosition();
            telemetry.addData("position", position);
            telemetry.update();
            if (gamepad1.a) {
                this.otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
            }
        }
    }
    public void initializeOtos () {
        this.otos.setLinearUnit(DistanceUnit.INCH);
        this.otos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        this.otos.setOffset(offset);
        this.otos.setLinearScalar(0);
        this.otos.setAngularScalar(0);


        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        this.otos.resetTracking();
        this.otos.setPosition(currentPosition);
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        this.otos.getVersionInfo(hwVersion, fwVersion);
    }
}

