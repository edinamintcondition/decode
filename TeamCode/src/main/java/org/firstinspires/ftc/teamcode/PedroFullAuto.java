package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import pedropathing.follower.Follower;
import pedropathing.path.PathChain;
import pedropathing.path.PathBuilder;
import pedropathing.path.pathgeneration.BezierLine;
import pedropathing.path.pathgeneration.Point;
import pedropathing.util.Constants;
import pedropathing.util.Pose;

@Autonomous(name = "Pedro Full Auto", group = "Autonomous")
public class PedroFullAuto extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        // Initialize Pedro follower
        follower = Constants.createFollower(hardwareMap);

        // Define starting pose (x, y, heading in radians)
        Pose startPose = new Pose(28.5, 128, Math.toRadians(180));
        follower.setStartingPose(startPose);

        // Build multiple paths
        PathChain scorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(60, 85)))
                .setLinearHeadingInterpolation(startPose.heading, Math.toRadians(135))
                .build();

        PathChain pickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(60, 85), new Point(40, 100)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        PathChain parkPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(40, 100), new Point(10, 130)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        // State machine for autonomous routine
        int state = 0;
        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    follower.followPath(scorePath);
                    state++;
                    break;
                case 1:
                    if (follower.isFinished()) {
                        follower.followPath(pickupPath);
                        state++;
                    }
                    break;
                case 2:
                    if (follower.isFinished()) {
                        follower.followPath(parkPath);
                        state++;
                    }
                    break;
                case 3:
                    if (follower.isFinished()) {
                        telemetry.addLine("Auto Complete");
                        telemetry.update();
                        requestOpModeStop();
                    }
                    break;
            }
            follower.update();
        }
    }
}
