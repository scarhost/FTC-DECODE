package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "basic test opmode + drive", group = "Test")
public class BasicTestDrive extends OpMode {

    // =========================
    // DRIVETRAIN (from MAIN_EXE)
    // =========================
    private DcMotor front_left, front_right, back_left, back_right;

    // Dashboard-tunable-style constants (kept simple; no dashboard import needed)
    private static final float DEADZONE  = 0.10f;
    private static final float SLOW_MULT = 0.40f;
    private static final float NORM_MULT = 1.00f;

    private boolean slowMode = false;
    private boolean leftStickPrev = false;
    private float speedMult = NORM_MULT;

    // =========================
    // BASIC TEST OPMODE SYSTEMS
    // =========================
    private DcMotor launcher;       // "launcher"
    private DcMotor turret_motor;   // "turret_motor"
    private DcMotor intake;         // "intake"
    private Servo  test_servo;      // "test_servo"

    // Battery
    private VoltageSensor batteryVoltageSensor;
    private static final double NOMINAL_VOLTAGE = 12.0;

    // Launcher
    private double launcherPower = 0.5;  // 0..1 pre-comp
    private boolean launcherOn = false;  // toggled by right stick click
    private boolean rStickBtnPrev = false;

    // Turret lock
    private boolean turretLocked = false;
    private boolean bPrev = false;

    // Turret encoder display
    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;

    // Intake (hold A)
    private double intakePower = 0.8;  // tunable with dpad up/down
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    // Kicker servo
    private static final double KICKER_ZERO = 0.2400;
    private static final double KICKER_ONE  = 0.5120;

    private boolean yPrev = false;
    private boolean lbPrev = false;
    private boolean rbPrev = false;

    private enum KickerState { HOME, GOING_OUT, RETURNING_HOME }
    private KickerState kickerState = KickerState.HOME;
    private final ElapsedTime kickerTimer = new ElapsedTime();

    private static final double KICK_OUT_DELAY_S  = 0.5;
    private static final double KICK_HOME_DELAY_S = 0.5;

    @Override
    public void init() {

        telemetry.addLine("basic test opmode + drive READY");
        telemetry.addLine("Drive: sticks = mecanum, left stick button = slow mode");
        telemetry.addLine("Launcher: right stick click = toggle, dpad L/R = power");
        telemetry.addLine("Turret: B = lock (BRAKE/FLOAT), encoder shown in telemetry");
        telemetry.addLine("Intake: hold A = on, dpad U/D = intake power");
        telemetry.addLine("Kicker: Y = auto kick+return+rumble, LB=ZERO, RB=ONE");
        telemetry.update();

        // ---- Drivetrain map ----
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        // ---- Drivetrain directions (from MAIN_EXE) ----
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- Drivetrain zero power ----
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Subsystems map ----
        launcher     = hardwareMap.get(DcMotor.class, "launcher");
        turret_motor = hardwareMap.get(DcMotor.class, "turret_motor");
        intake       = hardwareMap.get(DcMotor.class, "intake");
        test_servo   = hardwareMap.get(Servo.class, "test_servo");

        // ---- Launcher setup ----
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPower(0);

        // ---- Turret setup (encoder enabled) ----
        turret_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        turret_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret_motor.setPower(0);
        turret_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- Intake setup ----
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setPower(0);

        // ---- Servo setup ----
        test_servo.setPosition(KICKER_ZERO);
        kickerState = KickerState.HOME;
        kickerTimer.reset();

        // ---- Voltage sensor ----
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {

        // =========================
        // DRIVE (from MAIN_EXE)
        // =========================
        float y  = dead(gamepad1.left_stick_y);   // forward/back
        float x  = dead(gamepad1.left_stick_x);   // strafe
        float rx = dead(gamepad1.right_stick_x);  // rotate

        // Slow mode toggle
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
        // LAUNCHER (right stick click toggle) + DPAD L/R tuning
        // =========================
        boolean dpadLeftNow  = gamepad1.dpad_left;
        boolean dpadRightNow = gamepad1.dpad_right;

        if (dpadRightNow && !dpadRightPrev) launcherPower += 0.1;
        if (dpadLeftNow  && !dpadLeftPrev)  launcherPower -= 0.1;

        dpadLeftPrev  = dpadLeftNow;
        dpadRightPrev = dpadRightNow;

        launcherPower = clip01(launcherPower);

        boolean rStickBtnNow = gamepad1.right_stick_button;
        if (rStickBtnNow && !rStickBtnPrev) launcherOn = !launcherOn;
        rStickBtnPrev = rStickBtnNow;

        double rawLauncherCmd = launcherOn ? launcherPower : 0.0;
        launcher.setPower(compensate(rawLauncherCmd));

        // =========================
        // TURRET LOCK (B click) + encoder telemetry
        // =========================
        boolean bNow = gamepad1.b;
        if (bNow && !bPrev) {
            turretLocked = !turretLocked;
            if (turretLocked) {
                turret_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turret_motor.setPower(0);
            } else {
                turret_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                turret_motor.setPower(0);
            }
        }
        bPrev = bNow;

        // =========================
        // INTAKE (hold A) + DPAD U/D tuning
        // =========================
        boolean dpadUpNow   = gamepad1.dpad_up;
        boolean dpadDownNow = gamepad1.dpad_down;

        if (dpadUpNow && !dpadUpPrev) intakePower += 0.1;
        if (dpadDownNow && !dpadDownPrev) intakePower -= 0.1;

        dpadUpPrev = dpadUpNow;
        dpadDownPrev = dpadDownNow;

        intakePower = clip01(intakePower);

        boolean intakeHeld = gamepad1.a;
        intake.setPower(intakeHeld ? intakePower : 0.0);

        // =========================
        // KICKER SERVO (Y auto-cycle + rumble) + LB/RB manual
        // =========================
        boolean lbNow = gamepad1.left_bumper;
        boolean rbNow = gamepad1.right_bumper;
        boolean yNow  = gamepad1.y;

        // Manual overrides
        if (lbNow && !lbPrev) {
            kickerState = KickerState.HOME;
            test_servo.setPosition(KICKER_ZERO);
            kickerTimer.reset();
        }
        if (rbNow && !rbPrev) {
            kickerState = KickerState.HOME;
            test_servo.setPosition(KICKER_ONE);
            kickerTimer.reset();
        }
        lbPrev = lbNow;
        rbPrev = rbNow;

        // Auto-cycle trigger
        if (yNow && !yPrev) {
            kickerState = KickerState.GOING_OUT;
            test_servo.setPosition(KICKER_ONE);
            kickerTimer.reset();
        }
        yPrev = yNow;

        // State machine
        if (kickerState == KickerState.GOING_OUT) {
            if (kickerTimer.seconds() >= KICK_OUT_DELAY_S) {
                kickerState = KickerState.RETURNING_HOME;
                test_servo.setPosition(KICKER_ZERO);
                kickerTimer.reset();
            }
        } else if (kickerState == KickerState.RETURNING_HOME) {
            if (kickerTimer.seconds() >= KICK_HOME_DELAY_S) {
                kickerState = KickerState.HOME;
                gamepad1.rumble(1.0, 0.0, 200); // rumble after returning home
            }
        }

        // =========================
        // TELEMETRY
        // =========================
        double batteryV = batteryVoltageSensor.getVoltage();
        int turretTicks = turret_motor.getCurrentPosition();

        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Mode", slowMode ? "SLOW" : "NORMAL");
        telemetry.addData("SpeedMult", "%.2f", speedMult);

        telemetry.addLine("\n=== BATTERY ===");
        telemetry.addData("Voltage", "%.2f", batteryV);

        telemetry.addLine("\n=== LAUNCHER ===");
        telemetry.addData("ON?", launcherOn ? "YES" : "NO");
        telemetry.addData("Power (pre-comp)", "%.2f", launcherPower);
        telemetry.addData("Applied", "%.2f", compensate(rawLauncherCmd));

        telemetry.addLine("\n=== TURRET ===");
        telemetry.addData("Locked?", turretLocked ? "YES (BRAKE)" : "NO (FLOAT)");
        telemetry.addData("Encoder ticks", turretTicks);

        telemetry.addLine("\n=== INTAKE ===");
        telemetry.addData("Hold A?", intakeHeld ? "YES" : "NO");
        telemetry.addData("Power", "%.2f", intakePower);

        telemetry.addLine("\n=== KICKER ===");
        telemetry.addData("State", kickerState);
        telemetry.addData("ZERO", "%.4f", KICKER_ZERO);
        telemetry.addData("ONE", "%.4f", KICKER_ONE);
        telemetry.addData("Pos", "%.4f", test_servo.getPosition());

        telemetry.update();
    }

    // =========================
    // Helpers
    // =========================
    private float dead(float v) {
        return (Math.abs(v) < DEADZONE) ? 0f : v;
    }

    private double compensate(double rawPower) {
        double voltage = batteryVoltageSensor.getVoltage();
        if (voltage <= 0) return rawPower;
        double scaled = rawPower * (NOMINAL_VOLTAGE / voltage);
        if (scaled > 1.0) scaled = 1.0;
        if (scaled < -1.0) scaled = -1.0;
        return scaled;
    }

    private double clip01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}
