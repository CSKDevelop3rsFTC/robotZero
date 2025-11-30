package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BasicFlywheelPIDF", group = "Test")
public class newTuner extends OpMode {

    // Motor names must match your RC config
    private DcMotorEx m1, m2;

    // 5203 1:1: 28 ticks per rev at output shaft
    private static final double TICKS_PER_REV = 28.0;

    // PIDF constants – start with Kp and Kf, leave Ki/Kd = 0 at first
    @Configurable
    public static class constants {
        public static double Kp = 0.0019;
        public static double Ki = 0.0;
        public static double Kd = 0.0;
        public static double Kf = 0.000187;
        private static double TARGET_RPM = 3500.0;
    }

    // Target RPM you want to hold

    // state
    private int lastPos1 = 0;
    private int lastPos2 = 0;
    private boolean firstLoop = true;

    private double integral = 0.0;
    private double lastError = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        m1 = hardwareMap.get(DcMotorEx.class, "f1M");
        m2 = hardwareMap.get(DcMotorEx.class, "f2M");

        // make both spin same direction
        m1.setDirection(DcMotorSimple.Direction.FORWARD);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        // encoder + mode setup
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // let it coast when power = 0
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        timer.reset();
        firstLoop = true;
        integral = 0.0;
        lastError = 0.0;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        // compute dt
        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3;
        timer.reset();

        // measure current RPM (average of the two motors)
        double currentRPM = getAverageRPM(dt);

        // PIDF
        double error = constants.TARGET_RPM - currentRPM;

        // integral / derivative
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = constants.Kp * error + constants.Ki * integral + constants.Kd * derivative;
        double ff  = constants.Kf * constants.TARGET_RPM;

        double power = pid + ff;

        // clamp power
        if (power > 1.0) power = 1.0;
        if (power < 0.0) power = 0.0;  // usually no braking for flywheel

        m1.setPower(power);
        m2.setPower(power);

        // DRIVER HUB TELEMETRY
        telemetry.addData("Target RPM", "%.0f", constants.TARGET_RPM);
        telemetry.addData("Current RPM", "%.0f", currentRPM);
        telemetry.addData("Error", "%.0f", error);
        telemetry.addData("Power", "%.3f", power);
        telemetry.update();
    }

    private double getAverageRPM(double dt) {
        int p1 = m1.getCurrentPosition();
        int p2 = m2.getCurrentPosition();

        double rpm1, rpm2;

        if (firstLoop) {
            rpm1 = 0.0;
            rpm2 = 0.0;
            firstLoop = false;
        } else {
            rpm1 = ticksToRpm(p1 - lastPos1, dt);
            rpm2 = ticksToRpm(p2 - lastPos2, dt);
        }

        lastPos1 = p1;
        lastPos2 = p2;

        return 0.5 * (rpm1 + rpm2);
    }

    private static double ticksToRpm(int deltaTicks, double dt) {
        double revs = deltaTicks / TICKS_PER_REV;
        double rps  = revs / Math.max(dt, 1e-6);
        return rps * 60.0;
    }
}
