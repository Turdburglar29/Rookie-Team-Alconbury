
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
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants30630;
import pedroPathing.constants.LConstants30630;

@TeleOp(name = "Ai1player", group = "Examples")
public class Ai1Player extends OpMode {

    // ---------------- Pedro follower ----------------
    private Follower follower;
    private final Pose startPose = new Pose(30, 75, Math.toRadians(180)); // TeleOp start

    // ---------------- Goal (inches) ----------------
    private static final double GOAL_X = 10.0;
    private static final double GOAL_Y = 137.0;

    // ---------------- Shooter/Intake limits ----------------
    private static final int SHOOTER_MAX_TPS = 2000;   // REV UltraPlanetary max velocity
    private static final int SHOOTER_MIN_TPS = 1000;   // safe lower bound to avoid fluky shots
    private static final int INTAKE_VELOCITY = 1400;

    // Your existing presets (kept for non-tracking mode)
    private static final int bankVelocity = 1200;
    private static final int medVelocity  = 1375;
    private static final int farVelocity  = 1375;

    // ---------------- Hardware ----------------
    public static DcMotor intake;
    private DcMotor shooter1;      // TOP wheel (reversed direction in init)
    private DcMotor shooter2;      // BOTTOM wheel
    private DcMotor ballstopper;
    private RevBlinkinLedDriver lights;

    private final ElapsedTime parktimer = new ElapsedTime();
    private final ElapsedTime runtime = new ElapsedTime();

    // ---------------- Telemetry / math constants (unchanged) ----------------
    static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202
    static final double DRIVE_GEAR_REDUCTION = 19.2032;    // goBilda 5202
    static final double WHEEL_DIAMETER_INCHES = 3.77953;   // goBilda 5202
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // ---------------- Dynamic Shooter State ----------------
    // Distance (in) -> Top velocity (tps)
    private static final double[] DISTANCE_IN = new double[] { 24,  36,  48,  60,  72,  84,  96, 108 };
    private static final int[]    TOP_TPS     = new int[]    {750, 740, 700, 500, 400, 350, 300, 300};

    // Distance (in) -> Bottom offset (tps below top). Mirrors your TeleOp presets:
    //   close range ~600+, farther ~200 (backspin tapers off with distance)
    private static final int[] BOTTOM_DELTA_TPS = new int[]  { 1250, 13050, 1450, 1550, 1700, 1800, 1900, 2000 };

    // Shooter "ready" tolerances (tighten after tuning)
    private static final int TOL_TOP_TPS    = 20;
    private static final int TOL_BOTTOM_TPS = 20;

    private boolean trackingEnabled = true;  // Dynamic tracking ON by default
    private boolean prevSquare = false;

    private int lastTargetTop = 0;
    private int lastTargetBottom = 0;
    private double lastDistanceIn = 0.0;

    @Override
    public void init() {
        Constants.setConstants(FConstants30630.class, LConstants30630.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);  // TeleOp start pose
        telemetry.update();

        intake       = hardwareMap.get(DcMotor.class, "intake");
        shooter1     = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2     = hardwareMap.get(DcMotor.class, "shooter2");
        ballstopper  = hardwareMap.get(DcMotor.class, "ballstopper");
        lights       = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        ((DcMotorEx) shooter1).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ((DcMotorEx) shooter2).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballstopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Keep your original directions
        ballstopper.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE); // top
    }

    @Override
    public void init_loop() { /* waiting to start */ }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // ---------------- Drive with Pedro ----------------
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        // ---------------- Toggle tracking on/off ----------------
        boolean sq = gamepad1.square;
        if (sq && !prevSquare) trackingEnabled = !trackingEnabled;
        prevSquare = sq;

        // ---------------- Dynamic tracking (always computing when enabled) ----------------
        if (trackingEnabled) {
            shotPowerDynamic(); // compute & set target velocities continuously
        }

        // ---------------- Shooting / Intake controls ----------------
        // Driver presses any of Cross/Circle/Triangle to request a shot.
        boolean shootRequest = gamepad1.cross || gamepad1.circle || gamepad1.triangle;

        if (trackingEnabled) {
            // Dynamic mode: feed only when ready AND shoot button held
            boolean ready = shotReadyDynamic();

            if (shootRequest && ready) {
                intake.setPower(1);
                ballstopper.setPower(1);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                // Let bumpers control intake when not actively feeding a shot
                if (gamepad1.right_bumper) {
                    intake.setPower(1);
                } else if (gamepad1.left_bumper) {
                    intake.setPower(-1);
                } else {
                    // Stop only if not feeding
                    intake.setPower(0);
                }
                ballstopper.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            }

        } else {
            // ---------------- Original non-tracking button map ----------------
            if (parktimer.milliseconds() >= 110000) gamepad1.rumble(2000);

            /* short shot */
            if (gamepad1.cross) {
                ((DcMotorEx) shooter1).setVelocity(bankVelocity);
                ((DcMotorEx) shooter2).setVelocity(bankVelocity - 600);

                if ((((DcMotorEx) shooter1).getVelocity() >= bankVelocity - 25)
                        && (((DcMotorEx) shooter2).getVelocity() >= bankVelocity - 620)
                        && (((DcMotorEx) shooter1).getVelocity() <= bankVelocity + 15)
                        && (((DcMotorEx) shooter2).getVelocity() <= bankVelocity - 590)) {
                    intake.setPower(1);
                    ballstopper.setPower(1);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                } else {
                    intake.setPower(0);
                    ballstopper.setPower(0);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                }

            } /* mid shot */
            else if (gamepad1.circle) {
                ((DcMotorEx) shooter1).setPower(0);
                ((DcMotorEx) shooter2).setPower(0);
                if ((((DcMotorEx) shooter1).getVelocity() >= medVelocity - 25)
                        && (((DcMotorEx) shooter2).getVelocity() >= medVelocity - 660)
                        && (((DcMotorEx) shooter1).getVelocity() <= medVelocity + 15)
                        && (((DcMotorEx) shooter2).getVelocity() <= medVelocity - 630)) {
                    intake.setPower(1);
                    ballstopper.setPower(1);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
                } else {
                    intake.setPower(0);
                    ballstopper.setPower(0);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                }

            } /* long shot */
            else if (gamepad1.triangle) {
                ((DcMotorEx) shooter1).setVelocity(farVelocity);
                ((DcMotorEx) shooter2).setVelocity(farVelocity - 200);
                if ((((DcMotorEx) shooter1).getVelocity() >= farVelocity - 20)
                        && (((DcMotorEx) shooter2).getVelocity() >= farVelocity - 220)
                        && (((DcMotorEx) shooter1).getVelocity() >= farVelocity + 20)
                        && (((DcMotorEx) shooter2).getVelocity() <= farVelocity - 190)) {
                    intake.setPower(1);
                    ballstopper.setPower(1);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                } else {
                    intake.setPower(0);
                    ballstopper.setPower(0);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                }

            } /* intake bumper */
            else if (gamepad1.right_bumper) {
                ((DcMotorEx) intake).setVelocity(INTAKE_VELOCITY - 2400);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                if (((DcMotorEx) intake).getVelocity() >= INTAKE_VELOCITY - 2000) {
                    // keep running
                } else {
                    intake.setPower(0);
                }
            } else {
                // powers down all motors
                ((DcMotorEx) shooter1).setVelocity(0);
                ((DcMotorEx) shooter2).setVelocity(0);
                shooter1.setPower(0);
                shooter2.setPower(0);
                ballstopper.setPower(0);
                intake.setPower(0);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);

                // Right bumper intake
                if (gamepad1.right_bumper) {
                    intake.setPower(1);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                } else {
                    intake.setPower(0);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                }
                // Left bumper outtake
                if (gamepad1.left_bumper) {
                    intake.setPower(1); // (kept as your original code; change to -1 to truly outtake)
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                } else {
                    intake.setPower(0);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                }
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
            }
        }

        // ---------------- Telemetry ----------------
        telemetry.addData("Mode", trackingEnabled ? "DYNAMIC" : "PRESET");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Distance -> Goal (in)", lastDistanceIn);
        telemetry.addData("Target Top (tps)", lastTargetTop);
        telemetry.addData("Target Bottom (tps)", lastTargetBottom);
        telemetry.addData("Vel Top (tps)", ((DcMotorEx) shooter1).getVelocity());
        telemetry.addData("Vel Bottom (tps)", ((DcMotorEx) shooter2).getVelocity());
        telemetry.update();
    }

    // ---------------- Dynamic shooter functions ----------------

    /** Continuously compute & set target shooter velocities from current pose -> goal distance. */
    private void shotPowerDynamic() {
        Pose p = follower.getPose();
        double dx = p.getX() - GOAL_X;
        double dy = p.getY() - GOAL_Y;
        double distIn = Math.hypot(dx, dy);
        lastDistanceIn = distIn;

        int targetTop = clamp(interp(distIn, DISTANCE_IN, TOP_TPS), SHOOTER_MIN_TPS, SHOOTER_MAX_TPS);
        int delta     = clamp(interp(distIn, DISTANCE_IN, BOTTOM_DELTA_TPS), 0, SHOOTER_MAX_TPS);
        int targetBottom = clamp(targetTop - delta, SHOOTER_MIN_TPS / 2, SHOOTER_MAX_TPS);

        ((DcMotorEx) shooter1).setVelocity(targetTop);
        ((DcMotorEx) shooter2).setVelocity(targetBottom);

        lastTargetTop = targetTop;
        lastTargetBottom = targetBottom;
    }

    /** Returns true if both wheels are within tolerance of current dynamic targets. */
    private boolean shotReadyDynamic() {
        int vTopNow = (int)((DcMotorEx) shooter1).getVelocity();
        int vBotNow = (int)((DcMotorEx) shooter2).getVelocity();
        boolean topReady = Math.abs(vTopNow - lastTargetTop) <= TOL_TOP_TPS;
        boolean botReady = Math.abs(vBotNow - lastTargetBottom) <= TOL_BOTTOM_TPS;
        return topReady && botReady;
    }

    /** Linear interpolation helper for distance -> value. */
    private int interp(double x, double[] xs, int[] ys) {
        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
        for (int i = 0; i < xs.length - 1; i++) {
            double x0 = xs[i], x1 = xs[i + 1];
            if (x >= x0 && x <= x1) {
                double t = (x - x0) / (x1 - x0);
                return (int)Math.round(ys[i] + t * (ys[i + 1] - ys[i]));
            }
        }
        return ys[ys.length - 1];
    }

    private int clamp(int v, int lo, int hi) { return Math.max(lo, Math.min(hi, v)); }
}
