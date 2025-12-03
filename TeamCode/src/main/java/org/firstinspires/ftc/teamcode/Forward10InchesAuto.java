package org.firstinspires.ftc.teamcode;

import static com.google.blocks.ftcrobotcontroller.hardware.HardwareType.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.BNO055IMU;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;

@Autonomous(name = "Forward 10 Inches", group = "Test")
public class Forward10InchesAuto extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        /*//  Initialize IMU
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParams);*/

        //  Configure DriverEncoderConstants for GoBilda motors
        DriveEncoderConstants driveEncoderConstants = new DriveEncoderConstants();
        driveEncoderConstants.leftFrontMotorName="frontLeftMotor";
        driveEncoderConstants.rightFrontMotorName="frontRightMotor";
        driveEncoderConstants.leftRearMotorName="backLeftMotor";
        driveEncoderConstants.rightRearMotorName="backRightMotor";
               // .setTicksPerRev(537.6) // GoBilda 537 RPM motors
               // .setWheelRadius(1.8898) // 96mm wheels in inches
              //  .setTrackWidth(13.5); // Distance between left and right wheels in inches

        //  Create Localizer
        DriveEncoderLocalizer localizer = new DriveEncoderLocalizer(hardwareMap, driveEncoderConstants);

        //  Create Follower
        FollowerConstants followerConstants = new FollowerConstants();
        PathConstraints pathConstraints = new PathConstraints(0.8, 50, 1, 1); // Speed limits
        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .setLocalizer(localizer)
                .pathConstraints(pathConstraints)
                .build();

        //  Define start pose and target pose (10 inches forward)
        Pose startPose = new Pose(0, 0, 0); // Starting at origin
        Pose forwardPose = new Pose(10, 0, 0); // Move forward 10 inches

        follower.setStartingPose(startPose);

        //  Build path
        Path forwardPath = new Path(new BezierLine(startPose, forwardPose));
        forwardPath.setConstantHeadingInterpolation(0); // Keep heading straight

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        //  Follow path
        follower.followPath(forwardPath);

        //  Update loop until done
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.update();
        }

        telemetry.addLine("Path complete");
        telemetry.update();
    }
}
