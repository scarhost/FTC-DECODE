package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Pedro OTOS Pose Only", group = "Test")
public class PedroOTOSPoseOnly extends OpMode {

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

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        Pose startPose = USE_CUSTOM_START
                ? new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG))
                : new Pose(Constants.START_X, Constants.START_Y, Math.toRadians(Constants.START_HEADING_DEG));

        follower.setStartingPose(startPose);
        follower.update();

        panels.debug("Status", "Initialized (Pose Only)");
        panels.debug("StartPose", startPose);
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        Pose p = follower.getPose();

        // Reset pose on A press
        boolean aNow = gamepad1.a;
        if (ENABLE_RESET && aNow && !aPrev) {
            Pose resetPose = new Pose(RESET_X_IN, RESET_Y_IN, Math.toRadians(RESET_HEADING_DEG));
            follower.setPose(resetPose);
        }
        aPrev = aNow;

        panels.debug("X (in)", p.getX());
        panels.debug("Y (in)", p.getY());
        panels.debug("Heading (deg)", Math.toDegrees(p.getHeading()));

        panels.debug("OTOS Offset X (in)", Constants.OTOS_OFFSET_X_IN);
        panels.debug("OTOS Offset Y (in)", Constants.OTOS_OFFSET_Y_IN);
        panels.debug("Linear Scalar", Constants.LINEAR_SCALAR);
        panels.debug("Angular Scalar", Constants.ANGULAR_SCALAR);

        panels.update(telemetry);
    }
}
