package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import pedroPathing.Constants;
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutoRED2 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain toBalls;
        public PathChain pickUp;
        public PathChain ToDumpPlace;
        public PathChain ToMiddle;
        public PathChain Toparkplaceandpark;

        public Paths(Follower follower) {
            toBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(38.634, 60.098))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            pickUp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(38.634, 60.098), new Pose(16.585, 60.098))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ToDumpPlace = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.585, 60.098), new Pose(116.878, 131.902))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ToMiddle = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(116.878, 131.902), new Pose(71.805, 32.780))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Toparkplaceandpark = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(71.805, 32.780), new Pose(38.829, 32.976))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}

