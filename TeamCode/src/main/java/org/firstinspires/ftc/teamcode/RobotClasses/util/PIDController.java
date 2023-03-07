package org.firstinspires.ftc.teamcode.RobotClasses.util;

// If this doesn't work, try using  WPI Lib's implementation
// (https://github.com/wpilibsuite/allwpilib/blob/45590eea220923903606e47c7428939c3ce979e2/wpilibj/src/main/java/edu/wpi/first/wpilibj/controller/PIDController.java)

/**
 * PID Controller Class. Code in this class is based
 * on <a href = "https://gist.github.com/bradley219/5373998">this </a>
 * code (it's in cpp).
 */
public class PIDController {
    private double
            dt = 0.1, // loop interval time

            max = 1,
            min = -1,

            kP,
            kI,
            kD ,

            prevError = 0,
            integral = 0;

    public PIDController(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public PIDController() {
        // These are ok constants
        kP = 1.2;
        kI = .0001;
        kD = .1;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public void setDt(double dt) {
        this.dt = dt;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kI;
    }

    public void setOutputRange(double min, double max) {
        this.min = min;
        this.max = max;
    }

    /**
     * Calculates output
     * @param setpoint desired output
     * @param pv "process value", input value
     * @return calcuated output
     */
    public double calculate(double pv, double setpoint) {

        // Calculate error
        double error = setpoint - pv;

        // Proportional term
        double pOut = kP * error;

        // Integral term
        integral += error * dt;
        double iOut = kI * integral;

        // Derivative term
        double derivative = (error - prevError) / dt;
        double dOut = kD * derivative;

        // Calculate output
        double output = pOut + iOut + dOut;

        // Restrict output
        if(output > max) {
            output = max;
        }
        else if(output < min) {
            output = min;
        }

        // Save error to previous error
        prevError = error;

        return output;
    }

    public void reset() {
        prevError = 0;
    }

}
