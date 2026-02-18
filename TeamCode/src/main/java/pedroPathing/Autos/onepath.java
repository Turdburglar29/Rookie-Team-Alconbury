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

@Autonomous(name = "onepath", group = "Auto")
    public class onepath extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime slowDownTimer = new ElapsedTime();
    private static final int bankVelocity = 2400;
    private static final int medVelocity = 2075;
    private static final int farVelocity = 2400;
    private static final int intakeVelocity = 1400;
    public static DcMotor intake;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor ballstopper;
    private RevBlinkinLedDriver LED;

    double hue;
        static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
        static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;

        private int pathState;
    //Start point-----------------------------------------------------------------------------------
        private final Pose startPose = new Pose(20, 30, Math.toRadians(0));
    //line 1 ScorePreload 1 ------------------------------------------------------------------------
        private final Pose scorePose = new Pose(15, 25, Math.toRadians(50));
    //Line 3 Pickup 1-------------------------------------------------------------------------------
        private final Pose pickup1Pose = new Pose(28, 60, Math.toRadians(0));
        private final Pose pickup1CP1 = new Pose(70, 70, Math.toRadians(0));
    //line 4 Score 1 -------------------------------------------------------------------------------
        private final Pose score1Pose = new Pose(45, 50, Math.toRadians(45));
    //line 6 Pickup  2 -----------------------------------------------------------------------------
        private final Pose pickup2Pose = new Pose(22, 85, Math.toRadians(0));
        private final Pose pickup2CP1 = new Pose(75, 95, Math.toRadians(0));
        private final Pose pickup2CP2 = new Pose(30, 90, Math.toRadians(0));
    //line 7 Push Bar ------------------------------------------------------------------------------
        private final Pose pushBarPose = new Pose(17, 80, Math.toRadians(0));
        private final Pose pushBarCP1 = new Pose(25, 80, Math.toRadians(0));
    //line 8 Score  2 ------------------------------------------------------------------------------
        private final Pose score2Pose = new Pose(45, 50, Math.toRadians(50));
        private final Pose score2CP1 = new Pose(40,70, Math.toRadians(0));
        private final Pose score2CP2 = new Pose(40, 60, Math.toRadians(10));
    //line 9 Pickup  3------------------------------------------------------------------------------
        private final Pose pickup3Pose = new Pose(25, 105, Math.toRadians(0));
        private final Pose pickup3CP1 = new Pose(70, 105, Math.toRadians(0));
        private final Pose pickup3CP2 = new Pose(45, 105, Math.toRadians(0));
    //line 10 Score 3-------------------------------------------------------------------------------
        private final Pose score3Pose = new Pose(45, 50, Math.toRadians(50));
        private final Pose score3CP1 = new Pose(25, 90, Math.toRadians(0));
        private final Pose score3CP2 = new Pose(30, 65, Math.toRadians(10));
    //line 10 Park----------------------------------------------------------------------------------
        private final Pose park = new Pose(37, 75, Math.toRadians(0));
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
                        setPathState(17);

                break; // --------------------------------------Shoots balls--------------------------------------------

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

            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

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

            LED = hardwareMap.get(RevBlinkinLedDriver.class,"LED");
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ballstopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
        }

        @Override
        public void start() {
            opmodeTimer.resetTimer();
            setPathState(0);
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
        }

        @Override
        public void stop() {
        }
    }

