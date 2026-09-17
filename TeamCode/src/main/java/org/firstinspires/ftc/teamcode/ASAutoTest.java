package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "Test Auto", /*preselectTeleOp="Your Drive Code Here",*/ group="test")
// @Disabled
public class ASAutoTest extends ASAuto {
    // Values are in inches, 0-144 (72 is middle of the field)
    private final Pose startPose = new Pose(72+42, 72+52, Math.toRadians(0));
    private final Pose launchPose = new Pose(82, 86, Math.toRadians(135));
    private final Pose humanPose = new Pose(0, 0, Math.toRadians(0)); // Change name to whatever the human player zone is
    private final Pose parkPose = new Pose(0, 0, Math.toRadians(0)); //TODO

    public AS_Action[] getActions() {
        // The sequence of actions used to launch pollen/nectar
        AS_Action[] launchSequence = {
                // Get ready for launching
                // new ASSpinLauncher(this),

                // Move to shooting position
                new ASFollowPath(this, follower.pathBuilder() // Action that tells the robot to move along the given path
                        .addPath(new BezierLine( // Add a new bezier line to path (straight line)
                            follower::getPose, // Line starting position (where the robot starts)
                            launchPose // Line ending position (where the robot needs to go)
                            ))
                        .setLinearHeadingInterpolation(follower.getHeading(), launchPose.getHeading()) //TODO
                        .build()
                ),

                // Launch!
                // new ASWaitForLauncher(this),
                // new ASWait(this, 500),

                // Reset
                // new ASStopLauncher(this),
                // new ASStopIntake(this),
                // new ASStopPusher(this)
        };

        return new AS_Action[] {
                // ======================= AUTO START ======================= //

                // Init
                new ASSetStartingPose(this, startPose), // Tell the robot where it will start the auto

                // Launch!
                //new ASActionSequence(this, launchSequence),

                // Get ready to intake
                // new ASCloseStopper(this),
                // new ASSpinIntake(this),
                // new ASSpinPusher(this),

                // Grab line
                new ASFollowPath(this, follower.pathBuilder()
                        // Grab line
                        .addPath(new BezierCurve( // Add bezier curve to path (add curve)
                                launchPose, // Curve starting position
                                humanPose, // Curve ending position
                                new Pose(0, 0) // Curve control point
                        ))
                        .setConstantHeadingInterpolation(Math.toRadians(270)) // Towards pollen, wherever that is
                        // Return to hive
                        .addPath(new BezierCurve(
                                humanPose,
                                launchPose,
                                new Pose(0, 0) // Find control point
                        ))
                        .setLinearHeadingInterpolation(humanPose.getHeading(), launchPose.getHeading())
                        .setNoDeceleration()
                        .build()
                ),

                //new ASStopPusher(this),
                //new ASStopIntake(this),

                // Launch!
                //new ASActionSequence(this, launchSequence),

                // Move to parking zone
                new ASFollowPath(this, follower.pathBuilder()
                        .addPath(new BezierCurve(
                                launchPose,
                                new Pose(60, 90),
                                new Pose(60, 130)
                        ))
                        .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(50))
                        .build()),

                // ======================== AUTO END ======================== //
        };
    }
}