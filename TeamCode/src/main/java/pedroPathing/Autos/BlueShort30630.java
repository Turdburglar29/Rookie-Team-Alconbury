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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import pedroPathing.constants.FConstants30630;
import pedroPathing.constants.LConstants30630;

@Autonomous(name = "BlueShort", group = "Auto")
    public class BlueShort30630 extends OpMode {
    private static final int bankVelocity = 2400;
    private static final int medVelocity = 2200;
    private static final int farVelocity = 2400;
    private static final int intakeVelocity = 1400;
    public static DcMotor intake;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor ballstopper;
    private Servo releasespinner;
    private Servo sort1;
    private Servo sort2;
    private RevBlinkinLedDriver lights;
    private NormalizedColorSensor test_color;
    double hue;
    void setSafePower(DcMotor motor,double targetPower0){
        final double SLEW_RATE=0.2;
        double currentPower=motor.getPower();

        double desiredChange=currentPower;
        double limitedChange=Math.max(-SLEW_RATE,Math.min(desiredChange,SLEW_RATE));

        motor.setPower(currentPower += limitedChange);
    }
        private ElapsedTime runtime = new ElapsedTime();
        private ElapsedTime clipTimer = new ElapsedTime();
        static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
        static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        double startTime;
        private int pathState;
    //Start point-----------------------------------------------------------------------------------
        private final Pose startPose = new Pose(15, 120, Math.toRadians(180));
    //line 1 ScorePreload 1 ------------------------------------------------------------------------
        private final Pose scorePose = new Pose(37, 100, Math.toRadians(140));
    //Line 3 Pickup 1-------------------------------------------------------------------------------
        private final Pose pickup1Pose = new Pose(17, 85, Math.toRadians(180));
        private final Pose pickup1CP1 = new Pose(60, 80, Math.toRadians(180));
    //line 4 Score 1 -------------------------------------------------------------------------------
        private final Pose score1Pose = new Pose(37, 100, Math.toRadians(140));
    //line 6 Pickup  2 -----------------------------------------------------------------------------
        private final Pose pickup2Pose = new Pose(17, 60, Math.toRadians(180));
        private final Pose pickup2CP1 = new Pose(50, 60, Math.toRadians(180));
        private final Pose pickup2CP2 = new Pose(30, 60, Math.toRadians(180));
    //line 7 Push Bar ------------------------------------------------------------------------------
        private final Pose pushBarPose = new Pose(17, 70, Math.toRadians(180));
        private final Pose pushBarCP1 = new Pose(20, 70, Math.toRadians(180));
    //line 8 Score  2 ------------------------------------------------------------------------------
        private final Pose score2Pose = new Pose(37, 100, Math.toRadians(140));
        private final Pose score2CP1 = new Pose(25,70, Math.toRadians(180));
        private final Pose score2CP2 = new Pose(30, 85, Math.toRadians(160));
    //line 9 Pickup  3------------------------------------------------------------------------------
        private final Pose pickup3Pose = new Pose(17, 40, Math.toRadians(180));
        private final Pose pickup3CP1 = new Pose(50, 45, Math.toRadians(180));
        private final Pose pickup3CP2 = new Pose(30, 45, Math.toRadians(180));
    //line 10 Score 3-------------------------------------------------------------------------------
        private final Pose score3Pose = new Pose(37, 100, Math.toRadians(140));
        private final Pose score3CP1 = new Pose(25, 60, Math.toRadians(180));
        private final Pose score3CP2 = new Pose(30, 80, Math.toRadians(160));
    //line 10 Park----------------------------------------------------------------------------------
        private final Pose park = new Pose(37, 75, Math.toRadians(180));
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
                        setPathState(1);
                break; // -------------------------------------------------------------------------------------------
                case 1:
                        if (!follower.isBusy()) {
                        setPathState(2);
                        }
                break; // -------------------------------------------------------------------------------------------
                case 2:
                        follower.followPath(Pickup1, true);
                        setPathState(3);
                break; // -------------------------------------------------------------------------------------------
                case 3:
                        if (!follower.isBusy()) {
                            setPathState(4);
                        }
                break; // -------------------------------------------------------------------------------------------
                case 4:
                        follower.followPath(Score1, true);
                        setPathState(5);
                break; // -------------------------------------------------------------------------------------------
                case 5:
                        if (!follower.isBusy()) {
                            setPathState(6);
                        }
                break; // -------------------------------------------------------------------------------------------
                case 6:
                        follower.followPath(Pickup2, true);
                        setPathState(7);
                break; // -------------------------------------------------------------------------------------------
                case 7:
                        if (!follower.isBusy()) {
                            setPathState(8);
                        }
                break; // -------------------------------------------------------------------------------------------
                case 8:
                        follower.followPath(PushBar, true);
                        setPathState(9);
                break; // -------------------------------------------------------------------------------------------
                case 9:
                         if (!follower.isBusy()) {
                            setPathState(10);
                        }
                break; // -------------------------------------------------------------------------------------------
                case 10:
                        follower.followPath(Score2, true);
                        setPathState(11);
                break; // -------------------------------------------------------------------------------------------
                case 11:
                        if (!follower.isBusy()) {
                            setPathState(12);
                        }
                break; // -------------------------------------------------------------------------------------------
                case 12:
                        follower.followPath(Pickup3, true);
                        setPathState(13);
                break; // -------------------------------------------------------------------------------------------
                case 13:
                    if (!follower.isBusy()) {
                        setPathState(14);
                    }
                break; // -------------------------------------------------------------------------------------------
                case 14:
                    follower.followPath(Score3, true);
                    setPathState(15);
                break; // -------------------------------------------------------------------------------------------
                case 15:
                    if (!follower.isBusy()) {
                        setPathState(16);
                    }
                break; // -------------------------------------------------------------------------------------------
                case 16:
                    follower.followPath(Park, true);
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

            telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
            NormalizedRGBA colors = test_color.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());

            //Determining the amount of red, green, and blue
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);

            //Determining HSV and alpha
            telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
            telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
            telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

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
            test_color = hardwareMap.get(NormalizedColorSensor.class, "test_color");
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

