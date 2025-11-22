package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.Constants;
@Autonomous(name = "Five Feet Forward Auto", group = "Auto")
//@Disabled
public class BasicAutoPath extends OpMode {

    public PathChain FiveFeetForward;
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Pose startPose = new Pose(0,0);

    public void buildPaths() {
        FiveFeetForward = follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, new Pose(0.000, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }
    public void start() {
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
// These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

        // Feedback to Driver Hub for debugging
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void stop() {}
}
