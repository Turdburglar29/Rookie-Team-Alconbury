package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


/**
 * This is an example teleop that showcases movement and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "DaveTeleop", group = "Examples")
public class DaveTeleop extends OpMode {
    private Follower follower;
    private DcMotor RightLift;
    private DcMotor LeftLift;
    private DcMotor SpinIntake;
    private DcMotor IntakeExtend;
    private Servo BucketL;
    private Servo BucketR;
    private Servo intakepitchLeft;
    private Servo SpecimenClaw;
    private final Pose startPose = new Pose(0,0,0);



    private void Joystick_Control() { //code for joystick control intake and lift
        RightLift.setPower(gamepad2.left_stick_y + gamepad2.left_stick_x);
        LeftLift.setPower(gamepad2.left_stick_y + gamepad2.left_stick_x);
        IntakeExtend.setPower(-gamepad2.right_stick_y + gamepad2.right_stick_x);
        telemetry.update();
    }

    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.6898 ;   // goBilda 5202 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 19.2032 ;     // goBilda 5202 Gear ratio reduction
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // goBilda 5202 Wheel diameter
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.5;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        RightLift = hardwareMap.get(DcMotor.class, "RightLift");
        LeftLift = hardwareMap.get(DcMotor.class, "LeftLift");
        SpinIntake = hardwareMap.get(DcMotor.class, "SpinIntake");
        IntakeExtend = hardwareMap.get(DcMotor.class, "IntakeExtend");
        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        intakepitchLeft = hardwareMap.get(Servo.class, "intakepitchLeft");
        SpecimenClaw = hardwareMap.get(Servo.class, "SpecimenClaw");

        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SpinIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SpinIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SpinIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Path0",  "Starting at %7d :%7d",SpinIntake.getCurrentPosition(),
                RightLift.getCurrentPosition(),LeftLift.getCurrentPosition());
        telemetry.update();

        SpinIntake.setDirection(DcMotor.Direction.REVERSE);
        LeftLift.setDirection(DcMotor.Direction.REVERSE);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }
    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

        if (gamepad2.dpad_left) { // eats sample
            SpinIntake.setPower(0.6);
            Joystick_Control();
        }else {
                SpinIntake.setPower(0);
            }

        if (gamepad2.dpad_right) { // spits sample
            SpinIntake.setPower(-1);
            Joystick_Control();
        }

        if (gamepad2.right_bumper) {//brings in intake
            BucketL.setPosition(1);
            BucketR.setPosition(0);
            Joystick_Control();
        } else {
            IntakeExtend.setPower(-.1);
            Joystick_Control();
        }
        if (gamepad2.triangle) {//drops intake
            IntakeExtend.setPower(1);
            intakepitchLeft.setPosition(0.947);
            Joystick_Control();
        } else {
            IntakeExtend.setPower(-.1);
            Joystick_Control();
        }
        if (gamepad2.cross) {//brings in intake
            IntakeExtend.setPower(-1);
            intakepitchLeft.setPosition(0.2);
            Joystick_Control();
        } else {
            IntakeExtend.setPower(-.1);
            Joystick_Control();
        }
        if (gamepad2.left_bumper) {
            BucketL.setPosition(0.4);
            BucketR.setPosition(0.6);
            Joystick_Control();
        }
        if (gamepad2.dpad_up) {
            SpecimenClaw.setPosition(1);
            Joystick_Control();
        }
        if (gamepad2.dpad_down) {
            SpecimenClaw.setPosition(0.7);
            Joystick_Control();
        }
        if (gamepad2.start) {
            //encoderLift(1, 0.5, 0.5, 2);
           // sleep(1);

        }
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}