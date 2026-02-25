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
@Autonomous(name = "RedLong30630", group = "Auto")
    public class RedLong30630 extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime slowDownTimer = new ElapsedTime();
    private static final int firstFarVelocity = 1332;
    private static final int secondFarVelocity = 1337;
    private static final int thirdFarVelocity = 1345;
    public static DcMotor intake;
    private RevBlinkinLedDriver lights;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor ballstopper;
        static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
        static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        private Follower follower;
        private Timer pathTimer, opmodeTimer;
        private int pathState;
    //Start point-----------------------------------------------------------------------------------
        private final Pose startPose = new Pose(70, 0, Math.toRadians(90));
    //Score 1st shot ------------------------------------------------------------------------
        private final Pose scorePose = new Pose(69, 10, Math.toRadians(70));
    // Pickup 1-------------------------------------------------------------------------------
        private final Pose pickup1Pose = new Pose(110, 32, Math.toRadians(0));
        private final Pose pickup1CP1 = new Pose(100, 34, Math.toRadians(340));
    //Score 2nd shot -------------------------------------------------------------------------------
        private final Pose score1Pose = new Pose(69, 10, Math.toRadians(70));
    //Pickup  2 -----------------------------------------------------------------------------
        private final Pose pickup2Pose = new Pose(120, 30, Math.toRadians(320));
        private final Pose pickup2CP1 = new Pose(100, 34, Math.toRadians(320));
        private final Pose pickup2CP2 = new Pose(115, 32, Math.toRadians(320));
    //Push Bar ------------------------------------------------------------------------------
        private final Pose pushBarPose = new Pose(126, 10, Math.toRadians(320));
        private final Pose pushBarCP1 = new Pose(126, 15, Math.toRadians(320));
    //Score 3rd shot------------------------------------------------------------------------------
        private final Pose score2Pose = new Pose(69, 10, Math.toRadians(70));
        private final Pose score2CP1 = new Pose(100,10, Math.toRadians(67));
        private final Pose score2CP2 = new Pose(80, 10, Math.toRadians(69));
    //Park----------------------------------------------------------------------------------
        private final Pose park = new Pose(80, 35, Math.toRadians(0));
    //private PathChain ;
        private Path scorePreload,Pickup1,Score1,Pickup2,PushBar,Score2,Park;
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
            Park = new Path(new BezierCurve(new Point(score2Pose), new Point(park)));
            Park.setLinearHeadingInterpolation(score2Pose.getHeading(), park.getHeading());
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
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Intake Velocity", ((DcMotorEx) intake).getVelocity());
            telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter1).getVelocity());
            telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter2).getVelocity());
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
                    && (((DcMotorEx) shooter1).getVelocity() >= firstFarVelocity -5)
                    && (((DcMotorEx) shooter1).getVelocity() <= firstFarVelocity +20)
                    && (((DcMotorEx) shooter2).getVelocity() >= firstFarVelocity -205)
                    && (((DcMotorEx) shooter2).getVelocity() <= firstFarVelocity -190))  {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    intake.setPower(1);
                    ballstopper.setPower(1);
            }else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        }
    public void ShotCheck2() {
        if ((!follower.isBusy())
                && (((DcMotorEx) shooter1).getVelocity() >= secondFarVelocity -10)
                && (((DcMotorEx) shooter1).getVelocity() <= secondFarVelocity +13)
                && (((DcMotorEx) shooter2).getVelocity() >= secondFarVelocity -210)
                && (((DcMotorEx) shooter2).getVelocity() <= secondFarVelocity -187))  {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intake.setPower(1);
            ballstopper.setPower(1);
        }else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
    public void ShotCheck3() {
        if ((!follower.isBusy())
                && (((DcMotorEx) shooter1).getVelocity() >= thirdFarVelocity -10)
                && (((DcMotorEx) shooter1).getVelocity() <= thirdFarVelocity +13)
                && (((DcMotorEx) shooter2).getVelocity() >= thirdFarVelocity -210)
                && (((DcMotorEx) shooter2).getVelocity() <= thirdFarVelocity -187))  {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intake.setPower(1);
            ballstopper.setPower(1);
        }else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
        public void Shot1Power() {
            ((DcMotorEx) shooter1).setVelocity(firstFarVelocity);    //starts shooter
            ((DcMotorEx) shooter2).setVelocity(firstFarVelocity-200);
        }
        public void Shot2Power() {
            ((DcMotorEx) shooter1).setVelocity(secondFarVelocity);    //starts shooter
            ((DcMotorEx) shooter2).setVelocity(secondFarVelocity-200);
        }
        public void Shot3Power() {
            ((DcMotorEx) shooter1).setVelocity(thirdFarVelocity);    //starts shooter
            ((DcMotorEx) shooter2).setVelocity(thirdFarVelocity-200);
        }
        public void ShooterOff() {
            shooter1.setPower(0); //turns shooter off
            shooter2.setPower(0);
        }
    }