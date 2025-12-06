package org.firstinspires.ftc.teamcode.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name = "blueGoal", group = "Autonomous")
public class blueGoal extends OpMode {

    private Follower follower;
    private Timer pathTimer/*, actionTimer, opmodeTimer*/;
    private int pathState;
    private PathChain preload, intake, shoot1;

    private DcMotor intakeMotor;

    private final Pose startPose = new Pose(126, 60, Math.toRadians(45));
    private final Pose shoot0 = new Pose(84,12, Math.toRadians(45));
    private final Pose intakePose = new Pose(84,49,Math.toRadians(90));
    private final Pose shoot = new Pose(84,12,Math.toRadians(90));

    //paths
    private void buildPath() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot0))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot0.getHeading())
                .build();
        intake = follower.pathBuilder()
                .addPath(new BezierLine(shoot0,intakePose))
                .setLinearHeadingInterpolation(shoot0.getHeading(), intakePose.getHeading())
                .build();
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose,shoot))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shoot.getHeading())
                .build();

    }
    // Path state
    private void setPathState(int s) { pathState = s; pathTimer.resetTimer(); }

    private void pathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preload, true);
                setPathState(1); // settle then push
                break;
            case 1: // Run forward (start -> mid)
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    follower.followPath(intake,true);
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(shoot1,true);
                setPathState(3);
                break;
            case 3:
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    setPathState(4); // jump straight to done
                }
                break;

            case 4:
                // handled in loop() for stop
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        // opmodeTimer = new Timer();
        //  opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        intakeMotor     = hardwareMap.get(DcMotor.class, "intake");

        buildPath();
    }


    @Override
    public void start() {
        //opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update();
        pathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("t", follower.getCurrentTValue());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

    }
    @Override
    public void stop() {

    }
}