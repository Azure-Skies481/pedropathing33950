package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

// NOTE: adjust these imports if DriveEncoderConstants / Encoder are in a different package in your project
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(5);


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
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            // TODO: replace these with your actual motor/encoder names in the hardwareMap
            .rightFrontMotorName("frontrightMotor")   // e.g. "right_front_motor"
            .rightRearMotorName("backrightMotor")    // e.g. "right_rear_motor"
            .leftRearMotorName("backleftMotor")     // e.g. "left_rear_motor"
            .leftFrontMotorName("frontleftMotor")    // e.g. "left_front_motor"


            // TODO: set measured robot dimensions (inches)
            .robotWidth( /* ROBOT_WIDTH_INCHES / 12.0 )   // replace 12.0 with your measured width
            .robotLength( / ROBOT_LENGTH_INCHES / 12.0 ) // replace 12.0 with your measured length

            // TODO: replace the multipliers below with the values obtained from the tuners
            // Example placeholders (you must run tuners and replace these):
            .forwardTicksToInches( / forwardMultiplier / 1.0 )
            .strafeTicksToInches(  / strafeMultiplier  / 1.0 )
            .turnTicksToInches(    / turnMultiplier    */ 1.0 );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                // Register the drive encoder localizer so the follower uses your encoder-based localization
                .driveEncoderLocalizer(localizerConstants)

                .pathConstraints(pathConstraints)
                .build();
    }
}