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
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants30630;
import pedroPathing.constants.LConstants30630;

@Autonomous(name = "RedShort30630", group = "Auto")
public class RedShort30630 extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime slowDownTimer = new ElapsedTime();
    private static final int bankVelocity = 1200;
    private static final int medVelocity = 1400;
    private static final int farVelocity = 1325;
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
        private final Pose startPose = new Pose(93, 95, Math.toRadians(45));
    //line 1 ScorePreload 1 ------------------------------------------------------------------------
        private final Pose scorePose = new Pose(76, 87, Math.toRadians(50));
    //Line 3 Pickup 1-------------------------------------------------------------------------------
        private final Pose pickup1Pose = new Pose(96, 64, Math.toRadians(0));
        private final Pose pickup1CP1 = new Pose(65, 64, Math.toRadians(0));
    //line 4 Score 1 -------------------------------------------------------------------------------
        private final Pose score1Pose = new Pose(76, 87, Math.toRadians(50));
    //line 6 Pickup  2 -----------------------------------------------------------------------------
        private final Pose pickup2Pose = new Pose(99, 40, Math.toRadians(0));
        private final Pose pickup2CP1 = new Pose(60, 40, Math.toRadians(0));
        private final Pose pickup2CP2 = new Pose(65, 40, Math.toRadians(0));
    //line 7 Push Bar ------------------------------------------------------------------------------
        private final Pose pushBarPose = new Pose(85, 52, Math.toRadians(0));
        private final Pose pushBarCP1 = new Pose(74, 52, Math.toRadians(0));
    //line 8 Score  2 ------------------------------------------------------------------------------
        private final Pose score2Pose = new Pose(76, 87, Math.toRadians(50));
        private final Pose score2CP1 = new Pose(80,70, Math.toRadians(30));
        private final Pose score2CP2 = new Pose(85, 60, Math.toRadians(40));
    //line 9 Pickup  3------------------------------------------------------------------------------
        private final Pose pickup3Pose = new Pose(96, 17, Math.toRadians(0));
        private final Pose pickup3CP1 = new Pose(75, 50, Math.toRadians(0));
        private final Pose pickup3CP2 = new Pose(73, 18, Math.toRadians(0));

    //line 10 Score 3-------------------------------------------------------------------------------
        private final Pose score3Pose = new Pose(76, 87, Math.toRadians(50));
        private final Pose score3CP1 = new Pose(80,70, Math.toRadians(30));
        private final Pose score3CP2 = new Pose(85, 60, Math.toRadians(40));
    //line 10 Park----------------------------------------------------------------------------------
        private final Pose park = new Pose(74, 52, Math.toRadians(0));
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
                        ((DcMotorEx) shooter1).setVelocity(bankVelocity);    //starts shooter
                        ((DcMotorEx) shooter2).setVelocity(bankVelocity - 640);
                        setPathState(1);
                        shotTimer.reset();
                break; // --------------------------------------Shoots balls--------------------------------------------
                case 1:
                        if ((!follower.isBusy()) && (((DcMotorEx) shooter1).getVelocity() >= bankVelocity - 10) && (((DcMotorEx) shooter2).getVelocity() >= bankVelocity -650)) {
                            intake.setPower(1);
                            ballstopper.setPower(1);
                                }
                        if(shotTimer.milliseconds() > 4000) {
                            setPathState(2);
                        }
                break; // --------------------------------------Picks up 1st line ---------------------------------------
                case 2:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

                        follower.followPath(Pickup1, true);
                        shooter1.setPower(0); //turns shooter off
                        shooter2.setPower(0);
                        setPathState(3);
                        slowDownTimer.reset();
                break; // -------------------------------------------------------------------------------------------
                case 3:
                        if(slowDownTimer.milliseconds() > 1300) {
                            follower.setMaxPower(.29);
                            intake.setPower(1);
                        }
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
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
                        ballstopper.setPower(0);
                        shooter1.setPower(0); //turns shooter off
                        shooter2.setPower(0);
                        setPathState(5);
                        ((DcMotorEx) shooter1).setVelocity(bankVelocity);    //starts shooter
                        ((DcMotorEx) shooter2).setVelocity(bankVelocity - 640);
                        shotTimer.reset();


                    break; // --------------------------------Shots Balls-------------------------------------------
                case 5:
                        if ((!follower.isBusy()) && (((DcMotorEx) shooter1).getVelocity() >= bankVelocity - 10) && (((DcMotorEx) shooter2).getVelocity() >= bankVelocity -650)) {
                            intake.setPower(1);
                            ballstopper.setPower(1);
                        }
                        if(shotTimer.milliseconds() > 4000) {
                        setPathState(6);
                        }
                break; // -----------------------------------Picks up 2nd Line----------------------------------------
                case 6:
                        follower.followPath(Pickup2, true);
                        intake.setPower(0);
                        ballstopper.setPower(0);
                        shooter1.setPower(0); //turns shooter off
                        shooter2.setPower(0);
                        setPathState(7);
                        slowDownTimer.reset();
                break; // -------------------------------------------------------------------------------------------
                case 7:
                        if(slowDownTimer.milliseconds() > 1500) {
                            follower.setMaxPower(.27);
                            intake.setPower(1);
                         }
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
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
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        intake.setPower(0);
                        follower.setMaxPower(1);
                        follower.followPath(Score2, true);
                        ((DcMotorEx) shooter1).setVelocity(bankVelocity);    //starts shooter
                        ((DcMotorEx) shooter2).setVelocity(bankVelocity - 640);
                        setPathState(11);
                        shotTimer.reset();
                break; // --------------------------------Shots Balls--------------------------------------------
                case 11:
                        if ((!follower.isBusy()) && (((DcMotorEx) shooter1).getVelocity() >= bankVelocity - 10) && (((DcMotorEx) shooter2).getVelocity() >= bankVelocity -650)) {
                            intake.setPower(1);
                            ballstopper.setPower(1);
                        }
                        if(shotTimer.milliseconds() > 4000) {
                            setPathState(12);
                        }
                        slowDownTimer.reset();
                break; // -------------------------------Picks up 3rd Line--------------------------------------------
                case 12:
                        follower.followPath(Pickup3, true);
                        intake.setPower(0);
                        shooter1.setPower(0); //turns shooter off
                        shooter2.setPower(0);
                        setPathState(13);
                        slowDownTimer.reset();
                break; // -------------------------------------------------------------------------------------------
                case 13:
                    if(slowDownTimer.milliseconds() > 2000) {
                        follower.setMaxPower(.27);
                        intake.setPower(1);
                    }
                    if (!follower.isBusy()) {
                        intake.setPower(0);
                        setPathState(14);
                    }
                break; // -------------------------------------------------------------------------------------------
                case 14:
                    follower.setMaxPower(1);
                    setPathState(15);
                    shotTimer.reset();
                break; // -------------------------------------------------------------------------------------------
                case 15:
                    if (!follower.isBusy()) {
                    }
                    if(shotTimer.milliseconds() > 5000) {
                        setPathState(16);
                    }
                break; // -------------------------------------------------------------------------------------------
                case 16:
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
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Flywheel Velocity1", ((DcMotorEx) shooter1).getVelocity());
            telemetry.addData("Flywheel Velocity2", ((DcMotorEx) shooter2).getVelocity());
            telemetry.update();

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
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
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
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

