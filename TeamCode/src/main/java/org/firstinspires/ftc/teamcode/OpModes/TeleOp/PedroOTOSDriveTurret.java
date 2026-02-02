package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Pedro OTOS Drive + Turret", group = "TeleOp")
public class PedroOTOSDriveTurret extends OpMode {

    // =========================
    // PANELS: START POSE FOR THIS OPMODE
    // =========================
    public static boolean USE_CUSTOM_START = true;
    public static double START_X_IN = 0.0;
    public static double START_Y_IN = 0.0;
    public static double START_HEADING_DEG = 0.0;

    // Reset pose (BACK press)
    public static boolean ENABLE_RESET = true;
    public static double RESET_X_IN = 0.0;
    public static double RESET_Y_IN = 0.0;
    public static double RESET_HEADING_DEG = 0.0;

    // =========================
    // PANELS: GOAL (field coords)
    // =========================
    public static boolean USE_CONSTANTS_GOAL = true;
    public static double GOAL_X_IN = 0.0;
    public static double GOAL_Y_IN = 144.0;

    // =========================
    // PANELS: DRIVE (MATCHES YOUR WORKING BasicTestDrive FEEL)
    // =========================
    public static double DEADZONE = 0.10;
    public static double SLOW_MULT = 0.40;
    public static double NORM_MULT = 1.00;

    // If you ever need to flip stick directions without touching code:
    public static boolean INVERT_FWD_BACK = false;   // flips left_stick_y
    public static boolean INVERT_STRAFE   = false;   // flips left_stick_x
    public static boolean INVERT_ROTATE   = false;   // flips right_stick_x

    // =========================
    // PANELS: TURRET AUTO-AIM
    // Motor name is "turret"
    // =========================
    public static boolean TURRET_ENABLE = true;

    // If your turret encoder is reversed relative to positive power, flip this:
    public static boolean TURRET_INVERT_MOTOR = false;

    // Encoder conversion:
    // ticksPerRad = (ticksPerRev * gearRatio) / (2π)
    // targetTicks = targetAngleRad * ticksPerRad
    public static double TURRET_TICKS_PER_REV = 112.0; // start guess: 28 CPR * 4:1
    public static double TURRET_GEAR_RATIO = 1.0;

    // Turret soft limits (degrees) relative to turret zero (encoder=0)
    public static double TURRET_MIN_DEG = -160.0;
    public static double TURRET_MAX_DEG =  160.0;

    // PD control
    public static double TURRET_kP = 0.010;
    public static double TURRET_kD = 0.0008;

    // Power caps + deadband
    public static double TURRET_MAX_POWER = 0.60;
    public static double TURRET_HOLD_DEADBAND_TICKS = 8.0;

    // Manual nudge (adds on top of auto-aim)
    // Use triggers: RT nudges +, LT nudges -
    public static double TURRET_MANUAL_NUDGE_POWER = 0.25;

    // Toggle auto-aim button
    public static boolean TURRET_TOGGLE_WITH_B = true;

    // =========================
    // HARDWARE
    // =========================
    private DcMotor front_left, front_right, back_left, back_right;
    private DcMotorEx turret;

    // =========================
    // STATE
    // =========================
    private boolean slowMode = false;
    private boolean leftStickPrev = false;
    private double speedMult = NORM_MULT;

    private boolean resetPrev = false;

    private boolean bPrev = false;
    private boolean turretAutoAim = true;
    private double turretLastErrTicks = 0.0;

    // Pedro
    private TelemetryManager panels;
    private Follower follower;

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // Pedro follower for OTOS pose
        follower = Constants.createFollower(hardwareMap);

        Pose startPose = USE_CUSTOM_START
                ? new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG))
                : new Pose(Constants.START_X, Constants.START_Y, Math.toRadians(Constants.START_HEADING_DEG));

        follower.setStartingPose(startPose);
        follower.update();

        // ---- Drivetrain map (same names as your working file) ----
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        // ---- Drivetrain directions (EXACTLY like your working file) ----
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- Drivetrain zero power ----
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turret motor name you specified:
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        // ---- Turret setup ----
        turret.setDirection(TURRET_INVERT_MOTOR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretAutoAim = TURRET_ENABLE;

        panels.debug("Status", "Initialized: Drive + OTOS + Turret");
        panels.debug("StartPose", startPose);
        panels.debug("TurretAutoAim", turretAutoAim);
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        // Update Pedro pose
        follower.update();
        Pose p = follower.getPose();

        // -------------------------
        // Optional pose reset (BACK press)
        // -------------------------
        boolean resetNow = gamepad1.back;
        if (ENABLE_RESET && resetNow && !resetPrev) {
            follower.setPose(new Pose(RESET_X_IN, RESET_Y_IN, Math.toRadians(RESET_HEADING_DEG)));
        }
        resetPrev = resetNow;

        // -------------------------
        // DRIVE (same mixing + motor directions as your working BasicTestDrive)
        // -------------------------
        double y  = dead(gamepad1.left_stick_y, DEADZONE);     // forward/back
        double x  = dead(gamepad1.left_stick_x, DEADZONE);     // strafe
        double rx = dead(gamepad1.right_stick_x, DEADZONE);    // rotate

        if (INVERT_FWD_BACK) y = -y;
        if (INVERT_STRAFE)   x = -x;
        if (INVERT_ROTATE)   rx = -rx;

        boolean stickBtn = gamepad1.left_stick_button;
        if (stickBtn && !leftStickPrev) slowMode = !slowMode;
        leftStickPrev = stickBtn;
        speedMult = slowMode ? SLOW_MULT : NORM_MULT;

        double fl = y - x - rx;
        double fr = y + x + rx;
        double bl = y + x - rx;
        double br = y - x + rx;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        front_left.setPower(fl * speedMult);
        front_right.setPower(fr * speedMult);
        back_left.setPower(bl * speedMult);
        back_right.setPower(br * speedMult);

        // -------------------------
        // TURRET AUTO-AIM + LIMIT CLAMP
        // Buttons:
        //  - B toggles auto-aim (if enabled)
        //  - RT nudges +, LT nudges -
        // -------------------------
        if (TURRET_TOGGLE_WITH_B) {
            boolean bNow = gamepad1.b;
            if (bNow && !bPrev) turretAutoAim = !turretAutoAim;
            bPrev = bNow;
        } else {
            turretAutoAim = TURRET_ENABLE;
        }

        // Goal selection
        double goalX = USE_CONSTANTS_GOAL ? Constants.BLUE_GOAL_X : GOAL_X_IN;
        double goalY = USE_CONSTANTS_GOAL ? Constants.BLUE_GOAL_Y : GOAL_Y_IN;

        // Distance from robot to goal (field space)
        double dx = goalX - p.getX();
        double dy = goalY - p.getY();
        double distToGoal = Math.hypot(dx, dy);

        // Manual nudge from triggers
        double manual = 0.0;
        if (gamepad1.right_trigger > 0.05) manual += TURRET_MANUAL_NUDGE_POWER;
        if (gamepad1.left_trigger  > 0.05) manual -= TURRET_MANUAL_NUDGE_POWER;

        double turretCmdPower = 0.0;
        double turretTargetDegClamped = 0.0;
        double turretErrTicks = 0.0;

        // Recompute ticks-per-rad every loop so Panels edits apply immediately
        double ticksPerRad = (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO) / (2.0 * Math.PI);

        if (TURRET_ENABLE) {
            if (turretAutoAim) {
                // Field angle to goal
                double angleField = Math.atan2(dy, dx);

                // Desired turret angle relative to robot heading
                double desiredTurretAngleRad = wrapRadians(angleField - p.getHeading());

                // Clamp to soft limits -> "stays at limit until robot rotates"
                double minRad = Math.toRadians(TURRET_MIN_DEG);
                double maxRad = Math.toRadians(TURRET_MAX_DEG);
                double clampedRad = clamp(desiredTurretAngleRad, minRad, maxRad);
                turretTargetDegClamped = Math.toDegrees(clampedRad);

                // Convert target to ticks
                double targetTicks = clampedRad * ticksPerRad;

                // Current ticks
                double currentTicks = turret.getCurrentPosition();

                turretErrTicks = targetTicks - currentTicks;

                // PD (derivative is per-loop; good enough for now)
                double derr = turretErrTicks - turretLastErrTicks;
                turretLastErrTicks = turretErrTicks;

                double pd = (TURRET_kP * turretErrTicks) + (TURRET_kD * derr);

                // Deadband hold
                if (Math.abs(turretErrTicks) < TURRET_HOLD_DEADBAND_TICKS) {
                    pd = 0.0;
                }

                turretCmdPower = clamp(pd, -TURRET_MAX_POWER, TURRET_MAX_POWER);

                // Manual nudge adds on top
                turretCmdPower = clamp(turretCmdPower + manual, -1.0, 1.0);
            } else {
                // Manual only
                turretCmdPower = clamp(manual, -1.0, 1.0);
                turretLastErrTicks = 0.0;
            }

            turret.setPower(turretCmdPower);
        } else {
            turret.setPower(0.0);
            turretLastErrTicks = 0.0;
        }

        // -------------------------
        // TELEMETRY (Panels)
        // -------------------------
        panels.debug("=== POSE ===", "");
        panels.debug("X (in)", p.getX());
        panels.debug("Y (in)", p.getY());
        panels.debug("Heading (deg)", Math.toDegrees(p.getHeading()));

        panels.debug("=== GOAL ===", "");
        panels.debug("GoalX", goalX);
        panels.debug("GoalY", goalY);
        panels.debug("Dist->Goal (in)", distToGoal);

        panels.debug("=== DRIVE ===", "");
        panels.debug("Mode", slowMode ? "SLOW" : "NORMAL");
        panels.debug("SpeedMult", speedMult);

        panels.debug("=== TURRET ===", "");
        panels.debug("Enabled", TURRET_ENABLE);
        panels.debug("AutoAim", turretAutoAim);
        panels.debug("TargetDeg(clamped)", turretTargetDegClamped);
        panels.debug("ErrTicks", turretErrTicks);
        panels.debug("CmdPower", turretCmdPower);
        panels.debug("Ticks", turret.getCurrentPosition());

        panels.update(telemetry);
    }

    // =========================
    // Helpers
    // =========================
    private double dead(double v, double dz) {
        return (Math.abs(v) < dz) ? 0.0 : v;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double wrapRadians(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
