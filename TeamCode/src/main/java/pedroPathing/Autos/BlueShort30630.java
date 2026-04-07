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
import com.qualcomm.robotcore.util.ElapsedTime;
import pedroPathing.constants.FConstants30630;
import pedroPathing.constants.LConstants30630;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueShort", group = "Auto")
    public class BlueShort30630 extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime slowDownTimer = new ElapsedTime();
    private static final int firstBankVelocity = 1000;

    public static DcMotor intake;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor ballstopper;

    private RevBlinkinLedDriver lights;
    private Servo ballholder;

    void setSafePower(DcMotor motor,double targetPower0){
        final double SLEW_RATE=0.2;
        double currentPower=motor.getPower();

        double desiredChange=currentPower;
        double limitedChange=Math.max(-SLEW_RATE,Math.min(desiredChange,SLEW_RATE));

        motor.setPower(currentPower += limitedChange);
    }
        static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
        static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        double startTime;
        private int pathState;
    //Start point-----------------------------------------------------------------------------------
        private final Pose startPose = new Pose(6, 70, Math.toRadians(135));
    //line 1 ScorePreload 1 ------------------------------------------------------------------------
        private final Pose scorePose = new Pose(30, 56, Math.toRadians(135));
    //Line 3 Pickup 1-------------------------------------------------------------------------------
        private final Pose pickup1Pose = new Pose(13, 32, Math.toRadians(180));
        private final Pose pickup1CP1 = new Pose(55, 32, Math.toRadians(180));
    //line 4 Score 1 -------------------------------------------------------------------------------
        private final Pose score1Pose = new Pose(30, 56, Math.toRadians(135));
    //line 6 Pickup  2 -----------------------------------------------------------------------------
        private final Pose pickup2Pose = new Pose(0, 2, Math.toRadians(180));
        private final Pose pickup2CP1 = new Pose(55, 20, Math.toRadians(180));
        private final Pose pickup2CP2 = new Pose(40, 2, Math.toRadians(180));
    //line 7 Push Bar ------------------------------------------------------------------------------
        private final Pose pushBarPose = new Pose(29, 40, Math.toRadians(180));
        private final Pose pushBarCP1 = new Pose(31, 37, Math.toRadians(180));
    //line 8 Score  2 ------------------------------------------------------------------------------
        private final Pose score2Pose = new Pose(30, 56, Math.toRadians(135));
        private final Pose score2CP1 = new Pose(40,8, Math.toRadians(180));
        private final Pose score2CP2 = new Pose(25, 51, Math.toRadians(140));
    //line 9 Pickup  3------------------------------------------------------------------------------
        private final Pose pickup3Pose = new Pose(17, 47, Math.toRadians(180));
        private final Pose pickup3CP1 = new Pose(50, 52, Math.toRadians(180));
        private final Pose pickup3CP2 = new Pose(30, 52, Math.toRadians(180));
    //line 10 Score 3-------------------------------------------------------------------------------
        private final Pose score3Pose = new Pose(27, 53, Math.toRadians(135));
        private final Pose score3CP1 = new Pose(35, 22, Math.toRadians(180));
        private final Pose score3CP2 = new Pose(30, 53, Math.toRadians(160));
    //line 10 Park----------------------------------------------------------------------------------
        private final Pose park = new Pose(25, 7, Math.toRadians(180));
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
            Score2 = new Path(new BezierCurve(new Point(pickup2Pose), new Point(score2CP1), new Point(score2CP2), new Point(score2Pose)));
            Score2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading());
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
                    ballholder.setPosition(0.4);
                    Shot1Power();
                    ShotCheck1();
                    if(shotTimer.milliseconds() > 4000) {
                        setPathState(2);
                    }
                    break; // --------------------------------------Picks up 1st line ---------------------------------------
                case 2:
                    follower.followPath(Pickup1, true);
                    ShooterOff();
                    ballstopper.setPower(0);
                    ballholder.setPosition(0.4);
                    slowDownTimer.reset();
                    setPathState(3);
                    break; // -----------------------------------Slows Down to pickup-----------------------------------
                case 3:
                    if(slowDownTimer.milliseconds() > 1250) {
                        ballholder.setPosition(0);
                        follower.setMaxPower(.29);
                        intake.setPower(.65);
                        ballstopper.setPower(-.1);
                    }
                    intake.setPower(1); //turns intake on
                    if (!follower.isBusy()) {
                        follower.setMaxPower(1);
                        intake.setPower(0);
                        ballholder.setPosition(0.4);
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
                    ballholder.setPosition(0.4);
                    Shot2Power();
                    ShotCheck2();
                    if(shotTimer.milliseconds() > 4500) {
                        follower.followPath(Pickup2, true);
                        setPathState(6);
                    }
                    break; // -----------------------------------Picks up 2nd Line----------------------------------------
                case 6:
                        ballstopper.setPower(0);
                        ShooterOff();
                        slowDownTimer.reset();
                        intake.setPower(0);
                        ballholder.setPosition(0.4);
                        setPathState(7);
                    break; // ---------------------------------Turns and intakes corner---------------------------------------
                case 7:
                    if(slowDownTimer.milliseconds() > 1550) {
                        ballholder.setPosition(0);
                        follower.setMaxPower(.29);
                        intake.setPower(.65);
                        ballstopper.setPower(-.1);
                    }
                    if (!follower.isBusy()) {
                        intake.setPower(0);
                        follower.setMaxPower(1);
                        setPathState(8);
                    }
                    break; // ----------------------------Slows down for intake---------------------------------------
                case 8:
                    follower.followPath(Score2, true);
                    Shot3Power();
                    shotTimer.reset();
                    setPathState(9);
                    break; // --------------------------------Moves to Score 3rd shot--------------------------------------------
                case 9:
                    ballholder.setPosition(0.4);
                    Shot3Power();
                    ShotCheck3();
                    if(shotTimer.milliseconds() > 7000) {
                        intake.setPower(0);
                        ballstopper.setPower(0);
                        ShooterOff();
                        setPathState(10);
                    }
                    break; // --------------------------------3rd Shot-----------------------------------------------
                case 10:
                    follower.followPath(Park, true);
                    setPathState(11);
                    break; // -------------------------------------------------------------------------------------------
                case 11:
                    if (!follower.isBusy()) {
                        stop();
                        break; // -------------------------------------------------------------------------------------------
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
        ballholder = hardwareMap.get(Servo.class, "ballholder");
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
    public void ShotCheck1() {
        if ((!follower.isBusy())
                && (((DcMotorEx) shooter1).getVelocity() >= firstBankVelocity -20)
                && (((DcMotorEx) shooter1).getVelocity() <= firstBankVelocity +10)
                && (((DcMotorEx) shooter2).getVelocity() >= firstBankVelocity -190)
                && (((DcMotorEx) shooter2).getVelocity() <= firstBankVelocity -150))   {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intake.setPower(1);
            ballstopper.setPower(1);
        }else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
    public void ShotCheck2() {
        if ((!follower.isBusy())
                && (((DcMotorEx) shooter1).getVelocity() >= firstBankVelocity -15)
                && (((DcMotorEx) shooter1).getVelocity() <= firstBankVelocity +5)
                && (((DcMotorEx) shooter2).getVelocity() >= firstBankVelocity -285)
                && (((DcMotorEx) shooter2).getVelocity() <= firstBankVelocity -150))  {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intake.setPower(1);
            ballstopper.setPower(1);
        }else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
    public void ShotCheck3() {
        if ((!follower.isBusy())
                && (((DcMotorEx) shooter1).getVelocity() >= firstBankVelocity -15)
                && (((DcMotorEx) shooter1).getVelocity() <= firstBankVelocity +5)
                && (((DcMotorEx) shooter2).getVelocity() >= firstBankVelocity -285)
                && (((DcMotorEx) shooter2).getVelocity() <= firstBankVelocity -150)) {
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
        ((DcMotorEx) shooter1).setVelocity(firstBankVelocity);    //starts shooter
        ((DcMotorEx) shooter2).setVelocity(firstBankVelocity-250);
    }
    public void Shot3Power() {
        ((DcMotorEx) shooter1).setVelocity(firstBankVelocity);    //starts shooter
        ((DcMotorEx) shooter2).setVelocity(firstBankVelocity-250);
    }
    public void ShooterOff() {
        shooter1.setPower(0); //turns shooter off
        shooter2.setPower(0);
    }
}

