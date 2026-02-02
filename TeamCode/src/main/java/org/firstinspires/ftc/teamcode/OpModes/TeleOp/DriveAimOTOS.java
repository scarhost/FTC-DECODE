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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Drive + Aim + Launcher (OTOS)", group = "TeleOp")
public class DriveAimOTOS extends OpMode {

    // =========================
    // Panels "knobs" (EDITABLE)
    // =========================

    // ---- Device names (RC config) ----
    public static String TURRET_MOTOR_NAME  = "turret";
    public static String LAUNCHER_MOTOR_NAME = "launcher";
    public static String INTAKE_MOTOR_NAME   = "intake";
    public static String KICKER_SERVO_NAME   = "test_servo";

    // ---- Motor direction toggles (so you can reverse without redeploy) ----
    public static boolean TURRET_REVERSED   = false;
    public static boolean LAUNCHER_REVERSED = true;
    public static boolean INTAKE_REVERSED   = false;

    // ---- Drive feel ----
    public static double DEADBAND = 0.05;
    public static boolean FIELD_CENTRIC = true;

    // ---- Turret conversion & limits ----
    public static double TICKS_PER_DEG = 0.311;     // adjust to your real ratio
    public static double LIMIT_LEFT_DEG = 90;       // +left
    public static double LIMIT_RIGHT_DEG = 90;      // +right

    // ---- Turret PID (power control) ----
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0002;
    public static double kF = 0.0;

    // ---- Launcher ----
    public static double LAUNCHER_POWER_STEP = 0.05;    // you asked for 0.05 (editable)
    public static double LAUNCHER_POWER_MIN  = 0.0;
    public static double LAUNCHER_POWER_MAX  = 1.0;
    public static double NOMINAL_VOLTAGE     = 12.0;    // used for compensation
    public static boolean VOLTAGE_COMPENSATE_LAUNCHER = true;

    // ---- Intake ----
    public static double INTAKE_POWER = 0.80;           // hold A
    public static double INTAKE_POWER_REVERSE = 0.60;   // hold X (optional reverse)
    public static boolean ENABLE_INTAKE_REVERSE = true;

    // ---- Kicker servo positions + timing ----
    public static double KICKER_HOME = 0.2400;
    public static double KICKER_OUT  = 0.5120;

    public static double KICK_OUT_DELAY_S  = 0.5;
    public static double KICK_HOME_DELAY_S = 0.5;

    public static boolean KICK_RUMBLE_ON_DONE = true;
    public static int KICK_RUMBLE_MS = 200;

    // =========================
    // Internals
    // =========================
    private TelemetryManager panels;
    private Follower follower;

    private DcMotorEx turret;
    private DcMotorEx launcher;
    private DcMotorEx intake;
    private Servo kicker;

    private VoltageSensor batteryVoltageSensor;

    // Turret "zero"
    private int turretZeroTicks = 0;
    private double iTerm = 0;
    private double prevErr = 0;

    // Launcher state
    private double launcherPower = 0.50;
    private boolean launcherOn = false;
    private boolean rStickPrev = false;

    // Kicker state machine
    private enum KickerState { HOME, GOING_OUT, RETURNING_HOME }
    private KickerState kickerState = KickerState.HOME;
    private final ElapsedTime kickerTimer = new ElapsedTime();
    private boolean yPrev = false;
    private boolean lbPrev = false;
    private boolean rbPrev = false;

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(
                Constants.START_X,
                Constants.START_Y,
                Math.toRadians(Constants.START_HEADING_DEG)
        ));

        turret   = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);
        launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER_MOTOR_NAME);
        intake   = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        kicker   = hardwareMap.get(Servo.class, KICKER_SERVO_NAME);

        // Voltage sensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Apply direction toggles
        turret.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(LAUNCHER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Turret encoder reset and custom PID (RUN_WITHOUT_ENCODER)
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretZeroTicks = 0;

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPower(0);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setPower(0);

        kicker.setPosition(KICKER_HOME);
        kickerState = KickerState.HOME;
        kickerTimer.reset();

        panels.debug("Status", "Initialized");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        // ===== Drive via Pedro (teleop) =====
        double fwd = -dead(gamepad1.left_stick_y);
        double str = -dead(gamepad1.left_stick_x);
        double rot = -dead(gamepad1.right_stick_x);

        follower.setTeleOpDrive(fwd, str, rot, FIELD_CENTRIC);
        follower.update();
        Pose p = follower.getPose();

        // ===== Turret auto-aim at blue goal =====
        aimTurretAtBlueGoal(p);

        // ===== Distance to blue goal =====
        double dx = Constants.BLUE_GOAL_X - p.getX();
        double dy = Constants.BLUE_GOAL_Y - p.getY();
        double distToBlueGoal = Math.hypot(dx, dy); // same as sqrt(dx*dx+dy*dy)

        // ===== Launcher: toggle + power step (dpad L/R) =====
        boolean rStickNow = gamepad1.right_stick_button;
        if (rStickNow && !rStickPrev) launcherOn = !launcherOn;
        rStickPrev = rStickNow;

        if (edge(gamepad1.dpad_right)) launcherPower += LAUNCHER_POWER_STEP;
        if (edge(gamepad1.dpad_left))  launcherPower -= LAUNCHER_POWER_STEP;
        launcherPower = clamp(launcherPower, LAUNCHER_POWER_MIN, LAUNCHER_POWER_MAX);

        double launcherCmd = launcherOn ? launcherPower : 0.0;
        launcher.setPower(applyVoltageComp(launcherCmd));

        // ===== Intake: hold A forward, hold X reverse (optional) =====
        double intakeCmd = 0.0;
        if (gamepad1.a) intakeCmd = INTAKE_POWER;
        if (ENABLE_INTAKE_REVERSE && gamepad1.x) intakeCmd = -INTAKE_POWER_REVERSE;
        intake.setPower(clamp(intakeCmd, -1.0, 1.0));

        // ===== Kicker: Y auto-cycle, LB/RB manual =====
        boolean lbNow = gamepad1.left_bumper;
        boolean rbNow = gamepad1.right_bumper;
        boolean yNow  = gamepad1.y;

        // Manual overrides
        if (lbNow && !lbPrev) {
            kickerState = KickerState.HOME;
            kicker.setPosition(KICKER_HOME);
            kickerTimer.reset();
        }
        if (rbNow && !rbPrev) {
            kickerState = KickerState.HOME;
            kicker.setPosition(KICKER_OUT);
            kickerTimer.reset();
        }
        lbPrev = lbNow;
        rbPrev = rbNow;

        // Auto-cycle trigger
        if (yNow && !yPrev) {
            kickerState = KickerState.GOING_OUT;
            kicker.setPosition(KICKER_OUT);
            kickerTimer.reset();
        }
        yPrev = yNow;

        // State machine timing
        if (kickerState == KickerState.GOING_OUT) {
            if (kickerTimer.seconds() >= KICK_OUT_DELAY_S) {
                kickerState = KickerState.RETURNING_HOME;
                kicker.setPosition(KICKER_HOME);
                kickerTimer.reset();
            }
        } else if (kickerState == KickerState.RETURNING_HOME) {
            if (kickerTimer.seconds() >= KICK_HOME_DELAY_S) {
                kickerState = KickerState.HOME;
                if (KICK_RUMBLE_ON_DONE) {
                    gamepad1.rumble(1.0, 0.0, KICK_RUMBLE_MS);
                }
            }
        }

        // ===== Telemetry (Panels) =====
        panels.debug("Pose X", p.getX());
        panels.debug("Pose Y", p.getY());
        panels.debug("Heading deg", Math.toDegrees(p.getHeading()));
        panels.debug("Dist->BlueGoal", distToBlueGoal);

        panels.debug("Launcher ON", launcherOn);
        panels.debug("Launcher Pwr", launcherPower);
        panels.debug("Batt V", batteryVoltageSensor.getVoltage());

        panels.debug("Intake cmd", intakeCmd);

        panels.debug("Turret ticks", turret.getCurrentPosition());
        panels.debug("Turret deg est", ticksToDeg(turret.getCurrentPosition() - turretZeroTicks));

        panels.debug("Kicker state", kickerState);
        panels.debug("Kicker pos", kicker.getPosition());

        panels.update(telemetry);
    }

    // =========================
    // Turret aiming
    // =========================
    private void aimTurretAtBlueGoal(Pose robotPose) {
        double rx = robotPose.getX();
        double ry = robotPose.getY();
        double headingRad = robotPose.getHeading();

        double gx = Constants.BLUE_GOAL_X;
        double gy = Constants.BLUE_GOAL_Y;

        double angleToGoalRad = Math.atan2(gy - ry, gx - rx);
        double targetTurretRad = wrap(angleToGoalRad - headingRad);
        double targetTurretDeg = Math.toDegrees(targetTurretRad);

        targetTurretDeg = clamp(targetTurretDeg, -LIMIT_RIGHT_DEG, LIMIT_LEFT_DEG);
        int targetTicks = turretZeroTicks + degToTicks(targetTurretDeg);

        int currentTicks = turret.getCurrentPosition();
        double err = targetTicks - currentTicks;

        iTerm += err;
        double dTerm = err - prevErr;
        prevErr = err;

        double power = (kP * err) + (kI * iTerm) + (kD * dTerm) + kF;
        turret.setPower(clamp(power, -1.0, 1.0));
    }

    // =========================
    // Helpers
    // =========================
    private double dead(double v) {
        return (Math.abs(v) < DEADBAND) ? 0.0 : v;
    }

    // Simple "edge" helper for D-pad without writing 4 prev vars
    private boolean prevDpadL = false, prevDpadR = false;
    private boolean edge(boolean now) {
        // This helper is only used for left/right DPAD in this file.
        // If you later add up/down, make separate prevs or rewrite as a tiny Edge class.
        if (now == gamepad1.dpad_left) {
            boolean rising = now && !prevDpadL;
            prevDpadL = now;
            return rising;
        } else {
            boolean rising = now && !prevDpadR;
            prevDpadR = now;
            return rising;
        }
    }

    private double applyVoltageComp(double rawPower) {
        if (!VOLTAGE_COMPENSATE_LAUNCHER) return clamp(rawPower, -1.0, 1.0);
        double v = batteryVoltageSensor.getVoltage();
        if (v <= 0) return clamp(rawPower, -1.0, 1.0);
        double scaled = rawPower * (NOMINAL_VOLTAGE / v);
        return clamp(scaled, -1.0, 1.0);
    }

    private int degToTicks(double deg) {
        return (int) Math.round(deg * TICKS_PER_DEG);
    }

    private double ticksToDeg(int ticks) {
        if (TICKS_PER_DEG == 0) return 0;
        return ticks / TICKS_PER_DEG;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrap(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }
}
