package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Pedro OTOS Pose with Drive", group = "Test")
public class PedroOTOSPoseDRIVE extends OpMode {

    private TelemetryManager panels;
    private Follower follower;

    // =========================
    // START POSE FOR THIS OPMODE (PANELS EDITABLE)
    // =========================
    public static double START_X_IN = 0.0;
    public static double START_Y_IN = 0.0;
    public static double START_HEADING_DEG = 0.0;

    // If true, we use the values above. If false, we fall back to Constants.START_*
    public static boolean USE_CUSTOM_START = true;

    // =========================
    // RESET POSE (PANELS EDITABLE)
    // =========================
    public static boolean ENABLE_RESET = true;
    public static double RESET_X_IN = 0.0;
    public static double RESET_Y_IN = 0.0;
    public static double RESET_HEADING_DEG = 0.0;

    private boolean aPrev = false;

    // =========================
    // DRIVETRAIN (EXACT MATCH to BasicTestDrive)
    // =========================
    private DcMotor front_left, front_right, back_left, back_right;

    public static float DEADZONE  = 0.10f;
    public static float SLOW_MULT = 0.40f;
    public static float NORM_MULT = 1.00f;

    private boolean slowMode = false;
    private boolean leftStickPrev = false;
    private float speedMult = NORM_MULT;

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // Build Pedro follower (OTOS localizer runs inside)
        follower = Constants.createFollower(hardwareMap);

        Pose startPose = USE_CUSTOM_START
                ? new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG))
                : new Pose(Constants.START_X, Constants.START_Y, Math.toRadians(Constants.START_HEADING_DEG));

        follower.setStartingPose(startPose);
        follower.update();

        // ---- Drivetrain map ---- (same names as BasicTestDrive)
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        // ---- Drivetrain directions (EXACT SAME as BasicTestDrive) ----
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- Drivetrain zero power (EXACT SAME as BasicTestDrive) ----
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        panels.debug("Status", "Initialized (Drive + OTOS Pose)");
        panels.debug("StartPose", startPose);
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        // =========================
        // DRIVE (EXACT SAME math as BasicTestDrive)
        // =========================
        float y  = dead(gamepad1.left_stick_y);   // forward/back
        float x  = dead(gamepad1.left_stick_x);   // strafe
        float rx = dead(gamepad1.right_stick_x);  // rotate

        boolean stickBtn = gamepad1.left_stick_button;
        if (stickBtn && !leftStickPrev) slowMode = !slowMode;
        leftStickPrev = stickBtn;
        speedMult = slowMode ? SLOW_MULT : NORM_MULT;

        float fl = y - x - rx;
        float fr = y + x + rx;
        float bl = y + x - rx;
        float br = y - x + rx;

        float max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1f) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        front_left.setPower(fl * speedMult);
        front_right.setPower(fr * speedMult);
        back_left.setPower(bl * speedMult);
        back_right.setPower(br * speedMult);

        // =========================
        // UPDATE OTOS / POSE
        // =========================
        follower.update();
        Pose p = follower.getPose();

        // Reset pose on A press
        boolean aNow = gamepad1.a;
        if (ENABLE_RESET && aNow && !aPrev) {
            Pose resetPose = new Pose(RESET_X_IN, RESET_Y_IN, Math.toRadians(RESET_HEADING_DEG));
            follower.setPose(resetPose);
        }
        aPrev = aNow;

        // =========================
        // DISTANCE TO BLUE GOAL
        // =========================
        double dx = Constants.BLUE_GOAL_X - p.getX();
        double dy = Constants.BLUE_GOAL_Y - p.getY();
        double distToBlueGoal = Math.hypot(dx, dy);

        // =========================
        // TELEMETRY (Panels)
        // =========================
        panels.debug("Pose X (in)", p.getX());
        panels.debug("Pose Y (in)", p.getY());
        panels.debug("Heading (deg)", Math.toDegrees(p.getHeading()));

        panels.debug("Dist -> BlueGoal (in)", distToBlueGoal);

        panels.debug("OTOS Offset X (in)", Constants.OTOS_OFFSET_X_IN);
        panels.debug("OTOS Offset Y (in)", Constants.OTOS_OFFSET_Y_IN);
        panels.debug("Linear Scalar", Constants.LINEAR_SCALAR);
        panels.debug("Angular Scalar", Constants.ANGULAR_SCALAR);

        panels.debug("DriveMode", slowMode ? "SLOW" : "NORMAL");
        panels.debug("SpeedMult", speedMult);

        panels.update(telemetry);
    }

    private float dead(float v) {
        return (Math.abs(v) < DEADZONE) ? 0f : v;
    }
}
