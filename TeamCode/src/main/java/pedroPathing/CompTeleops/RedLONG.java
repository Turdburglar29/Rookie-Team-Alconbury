package pedroPathing.CompTeleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import pedroPathing.constants.FConstants30630;
import pedroPathing.constants.LConstants30630;

@TeleOp(name = "Red-LONG-T", group = "Examples")
public class RedLONG extends OpMode {

    private static final Logger log = LoggerFactory.getLogger(RedLONG.class);
    private Follower follower;
    private final ElapsedTime parktimer = new ElapsedTime();

    private static final double GOAL_X_RED =160;
    private static final double GOAL_Y_RED = 160;

    // Robot starts facing 270° (upfield)
    private final Pose startPose = new Pose(37, 10, Math.toRadians(0));

    public static DcMotor intake;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor ballstopper;
    private RevBlinkinLedDriver lights;
    private Servo ballholder;

    private final ElapsedTime runtime = new ElapsedTime();

    // =====================================================
    // 🔥 NEW: Heading offset system
    // =====================================================
    private double headingOffsetDeg = -180;   // field alignment correction
    private boolean lastLeft = false;
    private boolean lastRight = false;

    @Override
    public void init() {
        Constants.setConstants(FConstants30630.class, LConstants30630.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        ballstopper = hardwareMap.get(DcMotor.class, "ballstopper");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        ballholder = hardwareMap.get(Servo.class, "ballholder");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballstopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        ballstopper.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // =====================================================
        // 🔥 DPAD LEFT/RIGHT — Adjust heading offset by 2°
        // =====================================================
        boolean leftNow = gamepad1.dpad_left;
        boolean rightNow = gamepad1.dpad_right;

        if (leftNow && !lastLeft) headingOffsetDeg -= 2;
        if (rightNow && !lastRight) headingOffsetDeg += 2;

        lastLeft = leftNow;
        lastRight = rightNow;

        telemetry.addData("Heading Offset (deg)", headingOffsetDeg);

        double headingOffsetRad = Math.toRadians(headingOffsetDeg);


        // =====================================================
        // AUTO-AIM WHILE HOLDING DPAD DOWN
        // =====================================================
        double turnPower;

        if (gamepad1.dpad_down) {

            Pose pose = follower.getPose();
            double robotX = pose.getX();
            double robotY = pose.getY();

            // Apply heading offset to robot heading
            double robotHeading = pose.getHeading() + headingOffsetRad;

            double deltaX = GOAL_X_RED - robotX;

            double deltaY = GOAL_Y_RED - robotY;

            double targetAngle = -Math.atan2(deltaY, deltaX);

            targetAngle += Math.toRadians(90);

            double yCorrectionDeg = 5 - ((robotY - 80) * 0.2); // 10° over 50 units → 0.2°/unit
            double yCorrectionRad = Math.toRadians(yCorrectionDeg);
            targetAngle += yCorrectionRad;

            targetAngle += headingOffsetRad;

            // Wrap into [0, 2π)
            if (targetAngle < 0) targetAngle += 2 * Math.PI;
            if (targetAngle >= 2 * Math.PI) targetAngle -= 2 * Math.PI;

            double headingError = (targetAngle + -.43) - robotHeading;

            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

            double kP = 1.2;
            turnPower = headingError * kP;

            turnPower = Math.max(-0.7, Math.min(0.7, turnPower));

            telemetry.addData("Auto Aim Active", true);
            telemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngle));
            telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));

        } else {

            turnPower = -gamepad1.right_stick_x;
            telemetry.addData("Auto Aim Active", false);
        }

        // =====================================================
        // DRIVE (auto-aim overrides rotation only)
        // =====================================================
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnPower,
                false
        );

        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("Flywheel1 Velocity", ((DcMotorEx) shooter1).getVelocity());
        telemetry.addData("Flywheel2 Velocity", ((DcMotorEx) shooter2).getVelocity());

        telemetry.update();


        // =====================================================
        // YOUR ORIGINAL SHOOTING LOGIC (UNCHANGED)
        // =====================================================

        if (parktimer.milliseconds() >= 110000)
            gamepad1.rumble(2000);

        if (gamepad1.cross) {
            ((DcMotorEx) shooter1).setVelocity(1150);
            ((DcMotorEx) shooter2).setVelocity(1150 - 600);

            if (((DcMotorEx) shooter1).getVelocity() >= 1050) {
                intake.setPower(1);
                ballstopper.setPower(1);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            } else {
                intake.setPower(0);
                ballstopper.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }
        }

        else if (gamepad1.circle) {
            ((DcMotorEx) shooter1).setVelocity(1355);
            ((DcMotorEx) shooter2).setVelocity(1355 - 640);

            if (((DcMotorEx) shooter1).getVelocity() >= 1325) {
                intake.setPower(1);
                ballstopper.setPower(1);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            } else {
                intake.setPower(0);
                ballstopper.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }
        }

        else if (gamepad1.triangle) {
            ((DcMotorEx) shooter1).setVelocity(1400);
            ((DcMotorEx) shooter2).setVelocity(1060);

            if ((((DcMotorEx) shooter1).getVelocity() >= 1370)
                    && (((DcMotorEx) shooter2).getVelocity() >= 1040)) {

                intake.setPower(1);
                ballstopper.setPower(1);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            } else {
                intake.setPower(0);
                ballstopper.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }
        }

        else if (gamepad1.right_bumper) {
            ((DcMotorEx) intake).setVelocity(1400 - 2400);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);

            if (((DcMotorEx) intake).getVelocity() < 1400 - 2000) {
                intake.setPower(0);
            }
        }

        else {
            ((DcMotorEx) shooter1).setVelocity(0);
            ((DcMotorEx) shooter2).setVelocity(0);

            shooter1.setPower(0);
            shooter2.setPower(0);
            ballstopper.setPower(0);
            intake.setPower(0);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);

            if (gamepad1.right_bumper) intake.setPower(1);
            else intake.setPower(0);

            if (gamepad1.left_bumper) {
                intake.setPower(1);
                ballholder.setPosition(0);
            } else {
                ballholder.setPosition(0.4);
            }
        }
    }
}
