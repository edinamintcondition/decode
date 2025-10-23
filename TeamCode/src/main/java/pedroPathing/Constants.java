package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();
//          .mass(4);         //ROBOT MASS HERE

//    public static MecanumConstants driveConstants = new MecanumConstants()    //DRIVETRAIN CONSTANTS HERE
//            .maxPower(1)
//            .rightFrontMotorName("rf")
//            .rightRearMotorName("rr")
//            .leftRearMotorName("lr")
//            .leftFrontMotorName("lf")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

//    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()  ////DRIVE-ENCODER CONSTANTS HERE
//            .rightFrontMotorName("rf")
//            .rightRearMotorName("rr")
//            .leftRearMotorName("lr")
//            .leftFrontMotorName("lf")
//            .leftFrontEncoderDirection(Encoder.FORWARD)
//            .leftRearEncoderDirection(Encoder.FORWARD)
//            .rightFrontEncoderDirection(Encoder.FORWARD)
//            .rightRearEncoderDirection(Encoder.FORWARD);
//            .robotWidth()         //MEASUREMENT INFO HERE
//            .robotLength();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
//                 .mecanumDrivetrain(driveConstants)      //IMPLEMENT DRIVETRAIN CONSTANT HERE
//                .driveEncoderLocalizer(localizerConstants)        ////IMPLEMENT DRIVE-ENCODER CONSTANTS HERE
                .build();
    }
}
