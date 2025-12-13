package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.nio.file.Paths;

@Autonomous(name = "[BLUE]Pedro Pathing Autonomous", group = "Autonomous")
    @Configurable // Panels
    public class AutoBlue extends OpMode {

        private TelemetryManager panelsTelemetry; // Panels Telemetry instance
        private Timer pathTimer, actionTimer, opmodeTimer;
        public Follower follower; // Pedro Pathing follower instance
        private int pathState; // Current autonomous path state (state machine)
        private Paths paths; // Paths defined in the Paths class
        public PathChain Path1;
        private Pose startPose = new Pose(56.000, 8.000);

        @Override
        public void init() {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startPose);

            panelsTelemetry.debug("Status", "Initialized");
            panelsTelemetry.update(telemetry);

                pathTimer = new Timer();
                opmodeTimer = new Timer();
                opmodeTimer.resetTimer();

                buildPaths(follower);
        }
        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }

        @Override
        public void loop() {
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate();

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }

            public void buildPaths(Follower follower) {
                Path1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(startPose, new Pose(48.585, 95.415))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))
                        .build();
            }

        public void autonomousPathUpdate() {
            // Add your state machine Here
            // Access paths with paths.pathName
            // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
            switch(pathState) {
                case 0:
                    follower.followPath(Path1);
                    setPathState(-1);

            }
        }
    }
