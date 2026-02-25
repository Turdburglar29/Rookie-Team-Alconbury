package pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import pedroPathing.constants.FConstants30630;
import pedroPathing.constants.LConstants30630;

@Autonomous(name = "BlueLong30630", group = "Auto")
public class BlueLong30630 extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime slowDownTimer = new ElapsedTime();
    private static final int bankVelocity = 1200;
    private static final int medVelocity = 1400;
    private static final int farVelocity = 1350;
    private static final int maxVelocity = 1900; // 1900 is fastest
    private static final int intakeVelocity = 1400;
    public static DcMotor intake;

    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor ballstopper;
    private RevBlinkinLedDriver lights;
    double hue;
    static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

        private int pathState;
        //Start point-----------------------------------------------------------------------------------
        private final Pose startPose = new Pose(45, 0, Math.toRadians(90));
        //line 1 ScorePreload 1 ------------------------------------------------------------------------
        private final Pose scorePose = new Pose(49, 7, Math.toRadians(114));
        //Line 3 Pickup 1-------------------------------------------------------------------------------
        private final Pose pickup1Pose = new Pose(15, 25, Math.toRadians(180));
        private final Pose pickup1CP1 = new Pose(20, 20, Math.toRadians(180));
        //line 4 Score 1 -------------------------------------------------------------------------------
        private final Pose score1Pose = new Pose(47, 7, Math.toRadians(114));
        //line 6 Pickup  2 -----------------------------------------------------------------------------
        private final Pose pickup2Pose = new Pose(15, 20, Math.toRadians(250));
        private final Pose pickup2CP1 = new Pose(25, 25, Math.toRadians(250));
        private final Pose pickup2CP2 = new Pose(18, 22, Math.toRadians(250));
        //line 7 Push Bar ------------------------------------------------------------------------------
        private final Pose pushBarPose = new Pose(20, 10, Math.toRadians(250));
        private final Pose pushBarCP1 = new Pose(20, 15, Math.toRadians(250));
        //line 8 Score  2 ------------------------------------------------------------------------------
        private final Pose score2Pose = new Pose(47, 7, Math.toRadians(114));
        private final Pose score2CP1 = new Pose(20,7, Math.toRadians(113));
        private final Pose score2CP2 = new Pose(30, 7, Math.toRadians(113));
        //line 9 Pickup  3------------------------------------------------------------------------------
        private final Pose pickup3Pose = new Pose(25, 105, Math.toRadians(0));
        private final Pose pickup3CP1 = new Pose(45, 105, Math.toRadians(0));
        private final Pose pickup3CP2 = new Pose(45, 105, Math.toRadians(0));
        //line 10 Score 3-------------------------------------------------------------------------------
        private final Pose score3Pose = new Pose(65, 16, Math.toRadians(67));
        private final Pose score3CP1 = new Pose(65, 16, Math.toRadians(67));
        private final Pose score3CP2 = new Pose(65, 16, Math.toRadians(67));
        //line 10 Park----------------------------------------------------------------------------------
        private final Pose park = new Pose(25, 25, Math.toRadians(180));
    //  private PathChain ;-------------------------------------------------------------------------
        private Path scorePreload,  Pickup1,Score1,Pickup2,PushBar,Score2,Pickup3,Score3,Park;
//--------------------------------------------------------------------------------------------------
        public void buildPaths() {
//line 1 --------------------------------------------------------------------------------------------
            scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(scorePose)));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//Line 2 ---------------------------------------------------------------------------------------------------------------
            Pickup1 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup1CP1), new Point(pickup1Pose)));
            Pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());
//line 3 ---------------------------------------------------------------------------------------------------------------
            Score1 = new Path(new BezierCurve(new Point(pickup1Pose), new Point(score1Pose)));
            Score1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading());
//line 4 ----------------------------------------------------------------------------------------------------------------------------------
            Pickup2 = new Path(new BezierCurve(new Point(score1Pose), new Point(pickup2CP1), new Point(pickup2CP2), new Point(pickup2Pose)));
            Pickup2.setLinearHeadingInterpolation(score1Pose.getHeading(), pickup2Pose.getHeading());
//line 5 ----------------------------------------------------------------------------------------------------------------------------------
            PushBar = new Path(new BezierCurve(new Point(pickup2Pose), new Point(pushBarCP1), new Point(pushBarPose)));
            PushBar.setLinearHeadingInterpolation(pickup2Pose.getHeading(), pushBarPose.getHeading());
//line 6 ----------------------------------------------------------------------------------------------------------------------------------
            Score2 = new Path(new BezierCurve(new Point(pushBarPose), new Point(score2CP1), new Point(score2CP2), new Point(score2Pose)));
            Score2.setLinearHeadingInterpolation(pushBarPose.getHeading(), score2Pose.getHeading());
//line 7 ----------------------------------------------------------------------------------------------------------------------------------
            Pickup3 = new Path(new BezierCurve(new Point(score2Pose), new Point(pickup3CP1), new Point(pickup3CP2), new Point(pickup3Pose)));
            Pickup3.setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading());
//line 8 ----------------------------------------------------------------------------------------------------------------------------------
            Score3 = new Path(new BezierCurve(new Point(pickup3Pose), new Point(score3CP1), new Point(score3CP2), new Point(score3Pose)));
            Score3.setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading());
//line 9 ----------------------------------------------------------------------------------------------------------------------------------
            Park = new Path(new BezierCurve(new Point(score3Pose), new Point(park)));
            Park.setLinearHeadingInterpolation(score3Pose.getHeading(), park.getHeading());

        }
        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    follower.followPath(scorePreload, true);
                    ((DcMotorEx) shooter1).setVelocity(farVelocity);    //starts shooter
                    ((DcMotorEx) shooter2).setVelocity(farVelocity-200);
                    setPathState(1);
                    shotTimer.reset();
                    break; // --------------------------------------Shoots balls--------------------------------------------
                case 1:
                    ((DcMotorEx) shooter1).setVelocity(farVelocity);    //starts shooter
                    ((DcMotorEx) shooter2).setVelocity(farVelocity-200);
                    if ((!follower.isBusy())
                            && (((DcMotorEx) shooter1).getVelocity() >= farVelocity - 50)
                            && (((DcMotorEx) shooter1).getVelocity() <= farVelocity )
                            && (((DcMotorEx) shooter2).getVelocity() >= farVelocity -250)
                            && (((DcMotorEx) shooter2).getVelocity() <= farVelocity -200))  {
                        intake.setPower(1);
                        ballstopper.setPower(1);
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    if(shotTimer.milliseconds() > 4500) {
                        setPathState(2);
                    }
                    break; // --------------------------------------Picks up 1st line ---------------------------------------
                case 2:

                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    follower.followPath(Pickup1, true);
                    shooter1.setPower(0); //turns shooter off
                    shooter2.setPower(0);
                    ballstopper.setPower(0);
                    setPathState(3);
                    slowDownTimer.reset();
                    break; // -------------------------------------------------------------------------------------------
                case 3:
                    if(slowDownTimer.milliseconds() > 550) {
                        follower.setMaxPower(.29);
                        intake.setPower(1);
                    }
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    intake.setPower(1); //turns intake on
                    if (!follower.isBusy()) {
                        setPathState(4);
                    }
                    break; // -------------------------------------Moves to Score--------------------------------
                case 4:
                    follower.setMaxPower(1);
                    follower.followPath(Score1, true);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    intake.setPower(0);
                    shooter1.setPower(0); //turns shooter off
                    shooter2.setPower(0);
                    setPathState(5);
                    ((DcMotorEx) shooter1).setVelocity(farVelocity-250);    //starts shooter
                    ((DcMotorEx) shooter2).setVelocity(farVelocity-275);
                    shotTimer.reset();


                    break; // --------------------------------Shots Balls-------------------------------------------
                case 5:
                    ((DcMotorEx) shooter1).setVelocity(farVelocity);    //starts shooter
                    ((DcMotorEx) shooter2).setVelocity(farVelocity-200);
                    if ((!follower.isBusy())
                            && (((DcMotorEx) shooter1).getVelocity() >= farVelocity - 50)
                            && (((DcMotorEx) shooter1).getVelocity() <= farVelocity )
                            && (((DcMotorEx) shooter2).getVelocity() >= farVelocity -250)
                            && (((DcMotorEx) shooter2).getVelocity() <= farVelocity -200))  {
                        intake.setPower(1);
                        ballstopper.setPower(1);
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    if(shotTimer.milliseconds() > 4500) {
                        setPathState(6);
                    }
                    break; // -----------------------------------Picks up 2nd Line----------------------------------------
                case 6:
                    follower.followPath(Pickup2, true);
                    ballstopper.setPower(0);
                    intake.setPower(0);
                    shooter1.setPower(0); //turns shooter off
                    shooter2.setPower(0);
                    setPathState(66);
                    slowDownTimer.reset();
                    break; // -------------------------------------------------------------------------------------------
                case 66:
                    if (!follower.isBusy()){
                        follower.followPath(PushBar, true); //pushbar is actually second part of pick up balls in corner
                        setPathState(7);
                        slowDownTimer.reset();
                    }
                    break; // -------------------------------------------------------------------------------------------
                case 7:
                    if(slowDownTimer.milliseconds() > 100) {
                        follower.setMaxPower(.4);
                        intake.setPower(1);
                    }
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    if (!follower.isBusy()) {
                        setPathState(8);
                        intake.setPower(0);
                    }
                    break; // ----------------------------------Hits Push Bar-----------------------------------
                case 8:
                    follower.setMaxPower(.6);
                    /* follower.followPath(PushBar, true);*/
                    setPathState(9);
                    break; // -------------------------------------------------------------------------------------------
                case 9:

                    if (!follower.isBusy()) {
                        intake.setPower(0);
                        setPathState(10);
                    }
                    break; // --------------------------------Moves to Score--------------------------------------------
                case 10:
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    intake.setPower(0);
                    follower.setMaxPower(1);
                    follower.followPath(Score2, true);
                    ((DcMotorEx) shooter1).setVelocity(medVelocity);    //starts shooter
                    ((DcMotorEx) shooter2).setVelocity(medVelocity - 640);
                    setPathState(11);
                    shotTimer.reset();
                    break; // --------------------------------Shots Balls--------------------------------------------
                case 11:
                    ((DcMotorEx) shooter1).setVelocity(farVelocity);    //starts shooter
                    ((DcMotorEx) shooter2).setVelocity(farVelocity-200);
                    if ((!follower.isBusy())
                            && (((DcMotorEx) shooter1).getVelocity() >= farVelocity - 50)
                            && (((DcMotorEx) shooter1).getVelocity() <= farVelocity )
                            && (((DcMotorEx) shooter2).getVelocity() >= farVelocity -250)
                            && (((DcMotorEx) shooter2).getVelocity() <= farVelocity -200))  {
                        intake.setPower(1);
                        ballstopper.setPower(1);
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    if(shotTimer.milliseconds() > 4500) {
                        setPathState(12);
                    }
                    slowDownTimer.reset();
                    break; // -------------------------------Picks up 3rd Line--------------------------------------------
                case 12:
                    setPathState(13);
                    break; // -------------------------------------------------------------------------------------------
                case 13:
                    if (!follower.isBusy()) {
                        setPathState(14);
                    }
                    break; // -------------------------------------------------------------------------------------------
                case 14:
                    setPathState(15);
                    break; // -------------------------------------------------------------------------------------------
                case 15:
                    setPathState(16);
                    break; // -------------------------------------------------------------------------------------------
                case 16:
                    follower.followPath(Park, true);
                    shooter1.setPower(0); //turns shooter off
                    shooter2.setPower(0);
                    setPathState(17);
                    break; // -------------------------------------------------------------------------------------------
                case 17:
                    if (!follower.isBusy()) {
                        stop();
                    }
            }
        }
        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }

        @Override
        public void loop() {
            // These loop the movements of the robot
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());

            /*/Tells you Flywheel Velocity */
            telemetry.addData("Intake Velocity", ((DcMotorEx) intake).getVelocity());
            telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter1).getVelocity());
            telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter2).getVelocity());
            telemetry.update();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        @Override
        public void init() {
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();

            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();

            Constants.setConstants(FConstants30630.class, LConstants30630.class);
            follower = new Follower(hardwareMap);
            follower.setStartingPose(startPose);
            buildPaths();

            telemetry.update();
            intake = hardwareMap.get(DcMotor.class, "intake");
            shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            ballstopper = hardwareMap.get(DcMotor.class, "ballstopper");
            lights = hardwareMap.get(RevBlinkinLedDriver.class,"lights");

            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ballstopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
            ballstopper.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public void start() {
            opmodeTimer.resetTimer();
            setPathState(0);
        }

        @Override
        public void stop() {
        }
    }

