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
    private static final int firstBankVelocity = 1200;
    private static final int secondBankVelocity = 1200;
    private static final int thirdBankVelocity = 1200;
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
                case 0:// --------------------------------------Moves to 1st Shot--------------------------------------------
                    Shot1Power();
                    follower.followPath(scorePreload, true);
                    shotTimer.reset();
                    setPathState(1);
                    break; // --------------------------------------1st Shot--------------------------------------------
                case 1:
                    Shot1Power();
                    ShotCheck1();
                    if(shotTimer.milliseconds() > 4600) {
                        setPathState(2);
                    }
                    break; // --------------------------------------Picks up 1st line ---------------------------------------
                case 2:
                    follower.followPath(Pickup1, true);
                    ShooterOff();
                    ballstopper.setPower(0);
                    slowDownTimer.reset();
                    setPathState(3);
                    break; // -----------------------------------Slows Down to pickup-----------------------------------
                case 3:
                    if(slowDownTimer.milliseconds() > 900) {
                        follower.setMaxPower(.29);
                        intake.setPower(1);
                    }
                    intake.setPower(1); //turns intake on
                    if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        intake.setPower(0);
                        follower.followPath(Score1, true);
                        setPathState(4);
                    }
                    break; // -------------------------------------Moves to 2nd shot--------------------------------
                case 4:
                    follower.setMaxPower(1);
                    if (!follower.isBusy()) {
                        shotTimer.reset();
                        setPathState(5);
                    }
                    break; // --------------------------------2nd Shot-------------------------------------------
                case 5:
                    Shot2Power();
                    ShotCheck2();
                    if(shotTimer.milliseconds() > 4500) {
                        setPathState(6);
                    }
                    break; // -----------------------------------Picks up 2nd Line----------------------------------------
                case 6:
                    follower.followPath(Pickup2, true);
                    ballstopper.setPower(0);
                    intake.setPower(0);
                    ShooterOff();
                    slowDownTimer.reset();
                    setPathState(7);
                    break; // ---------------------------------Turns and intakes corner---------------------------------------
                case 7:
                    if (!follower.isBusy()){
                        follower.followPath(PushBar, true);
                        slowDownTimer.reset();
                        intake.setPower(1);
                        setPathState(8);
                    }
                    break; // ----------------------------Slows down for intake---------------------------------------
                case 8:
                    if(slowDownTimer.milliseconds() > 800) {
                        follower.setMaxPower(.5);
                        intake.setPower(1);
                    }
                    if (!follower.isBusy()) {
                        intake.setPower(1);
                        setPathState(9);
                    }
                    break; // --------------------------------Moves to Score 3rd shot--------------------------------------------
                case 9:
                    follower.setMaxPower(1);
                    follower.followPath(Score2, true);
                    Shot3Power();
                    shotTimer.reset();
                    setPathState(10);
                    break; // --------------------------------3rd Shot-----------------------------------------------
                case 10:
                    intake.setPower(0);
                    Shot3Power();
                    ShotCheck3();
                    if(shotTimer.milliseconds() > 8000) {
                        intake.setPower(0);
                        ballstopper.setPower(0);
                        setPathState(11);
                    }
                    break; // -------------------------------------------------------------------------------------------
                case 11:
                    follower.followPath(Park, true);
                    setPathState(12);
                    break; // -------------------------------------------------------------------------------------------
                case 12:
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
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
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
    public void ShotCheck1() {
        if ((!follower.isBusy())
                && (((DcMotorEx) shooter1).getVelocity() >= firstBankVelocity -5)
                && (((DcMotorEx) shooter1).getVelocity() <= firstBankVelocity +20)
                && (((DcMotorEx) shooter2).getVelocity() >= firstBankVelocity -205)
                && (((DcMotorEx) shooter2).getVelocity() <= firstBankVelocity -190))  {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intake.setPower(1);
            ballstopper.setPower(1);
        }else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
    public void ShotCheck2() {
        if ((!follower.isBusy())
                && (((DcMotorEx) shooter1).getVelocity() >= secondBankVelocity -10)
                && (((DcMotorEx) shooter1).getVelocity() <= secondBankVelocity +13)
                && (((DcMotorEx) shooter2).getVelocity() >= secondBankVelocity -210)
                && (((DcMotorEx) shooter2).getVelocity() <= secondBankVelocity -187))  {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intake.setPower(1);
            ballstopper.setPower(1);
        }else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
    public void ShotCheck3() {
        if ((!follower.isBusy())
                && (((DcMotorEx) shooter1).getVelocity() >= thirdBankVelocity -10)
                && (((DcMotorEx) shooter1).getVelocity() <= thirdBankVelocity +13)
                && (((DcMotorEx) shooter2).getVelocity() >= thirdBankVelocity -210)
                && (((DcMotorEx) shooter2).getVelocity() <= thirdBankVelocity -187))  {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intake.setPower(1);
            ballstopper.setPower(1);
        }else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
    public void Shot1Power() {
        ((DcMotorEx) shooter1).setVelocity(firstBankVelocity);    //starts shooter
        ((DcMotorEx) shooter2).setVelocity(firstBankVelocity-200);
    }
    public void Shot2Power() {
        ((DcMotorEx) shooter1).setVelocity(secondBankVelocity);    //starts shooter
        ((DcMotorEx) shooter2).setVelocity(secondBankVelocity-200);
    }
    public void Shot3Power() {
        ((DcMotorEx) shooter1).setVelocity(thirdBankVelocity);    //starts shooter
        ((DcMotorEx) shooter2).setVelocity(thirdBankVelocity-200);
    }
    public void ShooterOff() {
        shooter1.setPower(0); //turns shooter off
        shooter2.setPower(0);
    }
}

