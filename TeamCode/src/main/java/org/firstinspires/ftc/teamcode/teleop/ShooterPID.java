package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class ShooterPID {

    // 5203 1:1: 28 ticks per rev at output shaft
    private static final double TICKS_PER_REV = 28.0;

    // PIDF constants – SAME values you tuned
    @Configurable
    public static class constants {
        public static double Kp = 0.1;
        public static double Ki = 0.0;
        public static double Kd = 0.0;
        public static double Kf = 0.00023;

        // you can add more presets later if you want
        public static double DEFAULT_RPM = 3500.0;
    }

    private final DcMotorEx m1, m2;

    // state
    private int lastPos1 = 0;
    private int lastPos2 = 0;
    private boolean firstLoop = true;

    private double integral = 0.0;
    private double lastError = 0.0;
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;

    public ShooterPID(HardwareMap hw, String m1Name, String m2Name) {
        m1 = hw.get(DcMotorEx.class, m1Name);
        m2 = hw.get(DcMotorEx.class, m2Name);

        m1.setDirection(DcMotorSimple.Direction.FORWARD);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        firstLoop = true;
        integral = 0.0;
        lastError = 0.0;
    }

    // call this from TeleOp when you want a new speed
    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0.0, rpm);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    // call this ONCE PER LOOP from your TeleOp, pass in dt in seconds
    public void update(double dt) {
        if (dt <= 0) dt = 1e-3;

        currentRPM = getAverageRPM(dt);

        double error = targetRPM - currentRPM;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = constants.Kp * error + constants.Ki * integral + constants.Kd * derivative;
        double ff  = constants.Kf * targetRPM;

        double power = pid + ff;

        if (power > 1.0) power = 1.0;
        if (power < 0.0) power = 0.0;

        m1.setPower(power);
        m2.setPower(power);
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

    // convenience helper if you ever want to stop & coast
    public void stop() {
        targetRPM = 0.0;
        m1.setPower(0.0);
        m2.setPower(0.0);
        integral = 0.0;
        lastError = 0.0;
    }
}
