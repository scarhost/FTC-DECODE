package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;

    private int pathState = 0;
    private Paths paths;

    // Your REAL start (what you said)
    private static final Pose START_POSE = new Pose(
            21.883720930232545,
            122.69767441860465,
            Math.toRadians(144)
    );

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // 1) Tell Pedro where you start
        follower.setStartingPose(START_POSE);

        // 2) Build paths (uses follower.pathBuilder())
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("StartPose", START_POSE);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Start Path1 immediately when you press PLAY
        pathState = 1;
        follower.followPath(paths.Path1, true);
    }

    @Override
    public void loop() {
        // MUST be called every loop
        follower.update();

        // State machine advances when follower finishes current path
        autonomousPathUpdate();

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading(rad)", follower.getPose().getHeading());
        panelsTelemetry.debug("Heading(deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    /**
     * Simple sequential state machine:
     * Path1 -> Path2 -> ... -> Path10 -> done
     */
    private void autonomousPathUpdate() {
        switch (pathState) {

            case 1:
                if (!follower.isBusy()) { follower.followPath(paths.Path2, true); pathState = 2; }
                break;

            case 2:
                if (!follower.isBusy()) { follower.followPath(paths.Path3, true); pathState = 3; }
                break;

            case 3:
                if (!follower.isBusy()) { follower.followPath(paths.Path4, true); pathState = 4; }
                break;

            case 4:
                if (!follower.isBusy()) { follower.followPath(paths.Path5, true); pathState = 5; }
                break;

            case 5:
                if (!follower.isBusy()) { follower.followPath(paths.Path6, true); pathState = 6; }
                break;

            case 6:
                if (!follower.isBusy()) { follower.followPath(paths.Path7, true); pathState = 7; }
                break;

            case 7:
                if (!follower.isBusy()) { follower.followPath(paths.Path8, true); pathState = 8; }
                break;

            case 8:
                if (!follower.isBusy()) { follower.followPath(paths.Path9, true); pathState = 9; }
                break;

            case 9:
                if (!follower.isBusy()) { follower.followPath(paths.Path10, true); pathState = 10; }
                break;

            case 10:
                if (!follower.isBusy()) { pathState = 11; } // done
                break;

            default:
                // idle/done
                break;
        }
    }

    // ===== Paths live here =====
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        public Paths(Follower follower) {

            // Path1: start pose -> the point you showed in the screenshot
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(21.883720930232545, 122.69767441860465),
                            new Pose(60.8139534, 84.1395348)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
                    .build();

            // The rest are what you pasted earlier (kept the same)
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(60.814, 84.140),
                            new Pose(36.860, 84.140)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(36.860, 84.140),
                            new Pose(61.023, 84.140)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(135))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.023, 84.140),
                            new Pose(41.093, 59.977)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(41.093, 59.977),
                            new Pose(34.860, 60.070)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(34.860, 60.070),
                            new Pose(61.023, 84.140)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(135))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.023, 84.140),
                            new Pose(41.302, 35.488)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-180))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(41.302, 35.488),
                            new Pose(35.279, 35.442)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(35.279, 35.442),
                            new Pose(61.023, 84.140)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(135))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.023, 84.140),
                            new Pose(36.860, 84.140)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-180))
                    .build();
        }
    }
}
