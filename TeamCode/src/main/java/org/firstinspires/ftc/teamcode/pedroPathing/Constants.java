package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// NOTE: adjust these imports if DriveEncoderConstants / Encoder are in a different package in your project
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.2)
            .forwardZeroPowerAcceleration(-30.08)
            .lateralZeroPowerAcceleration(-52.71);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /*

LOCALIZER / ODOMETRY CONFIGURATION*
Replace the motor names below with the exact names you use in your robot's hardware map.
Replace encoder directions with the correct Encoder.FORWARD / Encoder.REVERSE for your wiring.
Measure your robot and replace ROBOT_WIDTH_INCHES and ROBOT_LENGTH_INCHES with the values you measured:
Length = distance between front and back wheels (in inches)
Width  = distance between left and right wheels (in inches)*
After running the forward / lateral / turn tuners, replace the multiplier placeholders with the
values produced by the tuners:
.forwardTicksToInches(forwardMultiplier)
.strafeTicksToInches(strafeMultiplier)
.turnTicksToInches(turnMultiplier)*
Examples below show placeholder values and TODO tags where you must fill in real values.*/
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD) //reversed

            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE) //reversed
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(61.33)
            .yVelocity(52.08)
            ;
    /*
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            // TODO: replace these with your actual motor/encoder names in the hardwareMap
            .rightFrontMotorName("frontrightMotor")   // e.g. "right_front_motor"
            .rightRearMotorName("backrightMotor")    // e.g. "right_rear_motor"
            .leftRearMotorName("backleftMotor")     // e.g. "left_rear_motor"
            .leftFrontMotorName("frontleftMotor")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD
            )
            .rightRearEncoderDirection(Encoder.FORWARD)// e.g. "left_front_motor"


            // TODO: set measured robot dimensions (inches)
            .robotWidth( 15.0)   // replace 12.0 with your measured width
            .robotLength( 13.2 ); // replace 12.0 with your measured length

            // TODO: replace the multipliers below with the values obtained from the tuners
            // Example placeholders (you must run tuners and replace these):

            .strafeTicksToInches( 1.0)//-3.741508 /
            .turnTicksToInches(1.0)   ; //0.3225698 /
            */

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1)
            .strafePodX(6.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);




    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                // Register the drive encoder localizer so the follower uses your encoder-based localization
                //.driveEncoderLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}