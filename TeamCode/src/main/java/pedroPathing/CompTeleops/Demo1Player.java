package pedroPathing.CompTeleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.R;

import pedroPathing.constants.FConstants30630;
import pedroPathing.constants.LConstants30630;

@TeleOp(name = "Demo1player", group = "Examples")
public class Demo1Player extends OpMode {
    private Follower follower;
    private ElapsedTime parktimer = new ElapsedTime();
    private static final int bankVelocity = 1200;
    private static final int medVelocity = 1375;
    private static final int farVelocity = 1375;
    private static final int maxVelocity = 1900; // 1900 is fastest
    private static final int intakeVelocity = 1400;
    private final Pose startPose = new Pose(0, 0, 0);

    public static DcMotor intake;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor ballstopper;
    private RevBlinkinLedDriver lights;


    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);




    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {
        Constants.setConstants(FConstants30630.class, LConstants30630.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.update();
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        ballstopper = hardwareMap.get(DcMotor.class, "ballstopper");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballstopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballstopper.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    }
    @Override
    public void init_loop() {/*This method is called continuously after Init while waiting to be started.*/
    }
    @Override
    public void start() {/*This method is called once at the start of the OpMode.*/
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {/*This is the main loop of the opmode and runs continuously after play*/
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        /*/Tells you Flywheel Velocity */


        telemetry.addData("Flywheel1 Velocity", ((DcMotorEx) shooter1).getVelocity());
        telemetry.addData("Flywheel2 Velocity", ((DcMotorEx) shooter2).getVelocity());
        telemetry.update();
//----------------------------------------------------------------------------
        if (parktimer.milliseconds() >= 110000)
            gamepad1.rumble(2000);
        /* short shot */
        if (gamepad1.cross) {
            ((DcMotorEx) shooter1).setVelocity(bankVelocity);/* sets Velocity */
            ((DcMotorEx) shooter2).setVelocity(bankVelocity - 600);
            if ((((DcMotorEx) shooter1).getVelocity() >= bankVelocity - 25)//low threshold
               && (((DcMotorEx) shooter2).getVelocity() >= bankVelocity - 620)
               && (((DcMotorEx) shooter1).getVelocity() <= bankVelocity + 15)//high threshold
               && (((DcMotorEx) shooter2).getVelocity() <= bankVelocity -590)) {
                intake.setPower(1);/* Pushes ball to shot */
                ballstopper.setPower(1);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            } else {
                intake.setPower(0);
                ballstopper.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }//----------------------------------------------------------------------------
        } /* mid shot */
        else if (gamepad1.circle) {
            ((DcMotorEx) shooter1).setVelocity(medVelocity);
            ((DcMotorEx) shooter2).setVelocity(medVelocity - 640);
            if ((((DcMotorEx) shooter1).getVelocity() >= medVelocity - 25)
               && (((DcMotorEx) shooter2).getVelocity() >= medVelocity - 660)
               && (((DcMotorEx) shooter1).getVelocity() <= medVelocity + 15)
               && (((DcMotorEx) shooter2).getVelocity() <= medVelocity -630)) {
                intake.setPower(1);
                ballstopper.setPower(1);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            } else {
                intake.setPower(0);
                ballstopper.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }//----------------------------------------------------------------------------
        } /* long shot */
        else if (gamepad1.triangle) {
            ((DcMotorEx) shooter1).setVelocity(farVelocity);
            ((DcMotorEx) shooter2).setVelocity(farVelocity - 200);
            if ((((DcMotorEx) shooter1).getVelocity() >= farVelocity -20)
               && (((DcMotorEx) shooter2).getVelocity() >= farVelocity - 220)
               && (((DcMotorEx) shooter1).getVelocity() >= farVelocity +20)
               && (((DcMotorEx) shooter2).getVelocity() <= farVelocity -190)) {
                intake.setPower(1);
                ballstopper.setPower(1);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            } else {
                intake.setPower(0);
                ballstopper.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }//----------------------------------------------------------------------------
        }/* intake */
        else if (gamepad1.right_bumper) {
            ((DcMotorEx) intake).setVelocity(intakeVelocity -2400);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            if (((DcMotorEx) intake).getVelocity() >= intakeVelocity - 2000) {
            } else {
                intake.setPower(0);
            }
        } else {/* powers down all motors */
            ((DcMotorEx) shooter1).setVelocity(0);
            ((DcMotorEx) shooter2).setVelocity(0);
            shooter1.setPower(0);
            shooter2.setPower(0);
            ballstopper.setPower(0);
            intake.setPower(0);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
//----------------------------------------------------------------------------
            if (gamepad1.right_bumper) {// intakes balls
                intake.setPower(1);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }else{
                intake.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }
//----------------------------------------------------------------------------
            if (gamepad1.left_bumper) {// outtakes balls
                intake.setPower(1);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            } else {
                intake.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }
//----------------------------------------------------------------------------
            if (gamepad1.dpad_left) {
                gamepad1.rumbleBlips(5);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            } else {
                gamepad1.stopRumble();
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }
            if (gamepad1.dpad_right) {
                gamepad1.rumble(1000);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            } else {
                gamepad1.stopRumble();
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }
            if (gamepad2.dpad_up) {
                gamepad1.rumble(1000);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            }
            if (gamepad2.dpad_down) {
            }
            if (gamepad2.back) {
            }
        }
    }
}


