package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {

    // =========================
    // FIELD / START POSE
    // =========================
    public static double START_X = 38.0;
    public static double START_Y = 134.2;
    public static double START_HEADING_DEG = 90.0;

    // Goal (you told me: 0, 144)
    public static double BLUE_GOAL_X = 0.0;
    public static double BLUE_GOAL_Y = 144.0;

    // =========================
    // OTOS SCALERS (your tuned numbers go here)
    // =========================
    // Example from what you gave earlier (keep or change):
    public static double LINEAR_SCALAR  = 48.0 / 51.36;
    public static double ANGULAR_SCALAR = 1800.0 / 1785.6;

    // =========================
    // OTOS OFFSET (Pedro coordinate frame)
    //
    // Pedro:
    // +X = forward
    // +Y = left
    //
    // You said sensor is:
    //  - 5.710309 in to the RIGHT  => Y negative
    //  - 8.879413 in DOWN/backward => X negative
    // =========================
    public static double OTOS_OFFSET_X_IN = -8.879413; // back
    public static double OTOS_OFFSET_Y_IN = -5.710309; // right


    // =========================
    // FOLLOWER CONSTANTS
    // =========================
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-43)
            .lateralZeroPowerAcceleration(-68)
            .mass(11.25)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.03, 0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.03, 0.001))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.08, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01));

    // =========================
    // PATH CONSTRAINTS
    // =========================
    public static PathConstraints pathConstraints = new PathConstraints(
            0.9925, 100, 2, 1
    );

    // =========================
    // DRIVE CONSTANTS
    // NOTE: Even pose-only testing still builds a follower with drivetrain.
    // Motor names MUST match RC config or you'll crash on init.
    // =========================
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("front_left")
            .leftRearMotorName("back_left")
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .useBrakeModeInTeleOp(true)
            // placeholders until you run velocity tuners
            .yVelocity(55)
            .xVelocity(68.57945);

    // =========================
    // OTOS LOCALIZER CONSTANTS (Pedro OTOS)
    //
    // IMPORTANT FIX:
    // .offset(...) expects a Pose (x,y,heading), NOT two doubles.
    // That's why your line was red.
    // =========================
    public static OTOSConstants otosConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(
                    OTOS_OFFSET_X_IN,
                    OTOS_OFFSET_Y_IN,
                    0.0
            ))
            .linearScalar(LINEAR_SCALAR)
            .angularScalar(ANGULAR_SCALAR);


    // =========================
    // BUILD FOLLOWER
    // =========================
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(otosConstants)
                .build();
    }
}
