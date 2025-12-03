/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "StarterBotTeleopWithSpeedAdjustment", group = "StarterBot")
//@Disabled
public class StarterBotTeleopWithSpeedAdjustment extends OpMode {

    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_MAX_VELOCITY = 90;
    final double LAUNCHER_TARGET_VELOCITY = 70;
    final double LAUNCHER_MIN_VELOCITY = 50;
    double launcherVelocity = LAUNCHER_TARGET_VELOCITY; // Dynamic velocity

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;

    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor pusher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private DcMotor intake = null;


    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        pusher = hardwareMap.get(DcMotorEx.class, "pusher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        pusher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 15));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotor.Direction.REVERSE);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        arcadeDrive();

        // Adjust launcher velocity using D-Pad
        // Preset speed modes
        if (gamepad1.dpad_left) {
            launcherVelocity = LAUNCHER_MIN_VELOCITY; // Low speed preset
        } else if (gamepad1.dpad_right) {
            launcherVelocity = LAUNCHER_MAX_VELOCITY; // High speed preset
        }
        if (gamepad1.dpad_up) {
            launcherVelocity += 3; // increase by 5 ticks/sec
        } else if (gamepad1.dpad_down) {
            launcherVelocity -= 3; // decrease by 5 ticks/sec
        }
        // Clamp velocity between 0 and LAUNCHER_MAX_VELOCITY
        launcherVelocity = Math.max(LAUNCHER_MIN_VELOCITY, Math.min(LAUNCHER_MAX_VELOCITY, launcherVelocity));
        telemetry.addData("Launcher Velocity", launcherVelocity);
        telemetry.addData("*************************", launcherVelocity);
        telemetry.addData("Press Y to Lunch with  speed -", launcherVelocity);
        telemetry.addData("Press X to Lunch with  speed -", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Press B to Stop everything ",null);
        telemetry.addData("Press A to Stop Launcher ",null);
        telemetry.addData("Press RB to Launcher  with Max speed",LAUNCHER_MAX_VELOCITY);
        telemetry.addData("Press dpad_left to set launcher Launcher with MIN speed",LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Press dpad_right to set launcher Launcher with MIN speed",LAUNCHER_MAX_VELOCITY);
        telemetry.addData("Press dpad_up to increase the speed",launcherVelocity);
        telemetry.addData("Press dpad_down to increase the speed",launcherVelocity);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            // launches with dynamic velocity
            telemetry.addData("Gamepad Y pressed, launcher velocity is",   launcher.getVelocity());
            launcher.setVelocity(launcherVelocity);
            intake.setPower(1.0);
            pusher.setPower(1.0);
            leftFeeder.setPower(1.0);
            rightFeeder.setPower(-1.0);
        } else if (gamepad1.b) { // stop flywheel
            telemetry.addData("Gamepad b pressed, everything stopped", true);
            launcher.setVelocity(STOP_SPEED);
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
            intake.setPower(STOP_SPEED);
            pusher.setPower(STOP_SPEED);
            telemetry.addData("Gamepad b pressed", true);
        } else if (gamepad1.x) { // reverse direction
            telemetry.addData("Reverse Direction", true);
            launcher.setVelocity(-30);
            intake.setPower(STOP_SPEED);
            pusher.setPower(-1.0);
            leftFeeder.setPower(-1.0);
            rightFeeder.setPower(1.0);

        }
        if (gamepad1.a) {
            telemetry.addData("Gamepad a pressed, launcher is stopped", true);
            launcher.setVelocity(STOP_SPEED);
            intake.setPower(1.0);
            pusher.setPower(1.0);
            leftFeeder.setPower(1.0);
            rightFeeder.setPower(-1.0);
            telemetry.addData("Gamepad a pressed, launcher is stopped", true);
        }
        if (gamepad1.rightBumperWasPressed()) {
            telemetry.addData("rightBumperWasPressed", true);
            telemetry.addData("rightBumperWasPressed, launches with speed ", LAUNCHER_MAX_VELOCITY);
            launcher.setVelocity(LAUNCHER_MAX_VELOCITY);
            intake.setPower(1.0);
            pusher.setPower(1.0);
            leftFeeder.setPower(1.0);
            rightFeeder.setPower(-1.0);
            telemetry.addData("speed is", launcher.getVelocity());
         //   launcher.setVelocity(launcherVelocity);
            telemetry.addData("rightBumperWasPressed, launches with speed ", LAUNCHER_MAX_VELOCITY);
        }
        if (gamepad1.leftBumperWasPressed()) {
            telemetry.addData("leftBumperWasPressed", LAUNCHER_TARGET_VELOCITY);
            launcher.setVelocity(launcherVelocity);
            intake.setPower(STOP_SPEED);
            pusher.setPower(STOP_SPEED);
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
            telemetry.addData("speed is", launcher.getVelocity());
        }

        /*
         * Show the state and motor powers
         */
        telemetry.addData("motorSpeed ", launcher.getVelocity());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    void arcadeDrive(){
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.update();
    }
}
