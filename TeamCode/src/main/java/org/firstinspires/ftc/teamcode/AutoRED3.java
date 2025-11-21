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
public class AutoRED3 extends OpMode {

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

        public PathChain tozheballs;
        public PathChain collectzheballs;
        public PathChain drivetozhepillarthingy;
        public PathChain backtozhemiddle;
        public PathChain parkzherobot;

        public Paths(Follower follower) {
            tozheballs = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(36.878, 83.707))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            collectzheballs = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.878, 83.707), new Pose(14.244, 83.707))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            drivetozhepillarthingy = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.244, 83.707), new Pose(119.415, 128.195))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            backtozhemiddle = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(119.415, 128.195), new Pose(63.610, 32.390))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            parkzherobot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(63.610, 32.390), new Pose(39.024, 32.195))
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

