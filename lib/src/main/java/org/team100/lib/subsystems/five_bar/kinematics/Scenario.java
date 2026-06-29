package org.team100.lib.subsystems.five_bar.kinematics;

/**
 * Details of a particular geometry and motor combination.
 */
public class Scenario {
    String name;
    // link lengths, meters
    public double a1;
    public double a2;
    public double a3;
    public double a4;
    public double a5;
    // position of P1, meters
    public double x1;
    public double y1;
    // // reduction
    // double ratio;
    // // stall torque, Nm
    // double Tmax;
    // // the work envelope
    // double w;
    // double h;
    public double xcenter;
    public double ycenter;

    /** Position of P1 */
    public Point P1() {
        return new Point(x1, y1);
    }

    /** Position of P5 */
    public Point P5() {
        double x5 = x1 - a5;
        double y5 = y1;
        return new Point(x5, y5);
    }
    // double xmin;
    // double xmax;
    // double ymin;
    // double ymax;

    // public double right() {
    // return xcenter + w / 2;
    // }

    // public double left() {
    // return xcenter - w / 2;
    // }

    // public double top() {
    // return ycenter + h / 2;
    // }

    // public double bottom() {
    // return ycenter - h / 2;
    // }
}
