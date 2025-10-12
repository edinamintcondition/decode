package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.acmerobotics.roadrunner.CompositePosePath;
import com.acmerobotics.roadrunner.PathBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name = "rrTestAuto", group = "StarterBot")
public class rrTestAuto extends LinearOpMode {
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;

    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    public static Pose2d startingPosition = new Pose2d(0,0,0);
    public static Vector2d waypoint1 = new Vector2d(15,15);
    public static Vector2d waypoint2 =new Vector2d(20,20)
    public static Double eps = 1.0;

    ElapsedTime feederTimer = new ElapsedTime();
   StarterBotTeleop Robot =new StarterBotTeleop();
    @Override
    public void runOpMode() throws InterruptedException {
     Robot.init();
     waitForStart();
     //I don't know what the first part of the code does
        List<CompositePosePath> path = new PathBuilder(startingPosition,eps)
                .splineTo(waypoint1,0)
                .lineToX(40)
                .lineToY(4)
                .build();
    }
}
