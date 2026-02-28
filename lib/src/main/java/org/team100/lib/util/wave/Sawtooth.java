package org.team100.lib.util.wave;

import java.util.function.DoubleUnaryOperator;

/**
 * A sawtooth wave.
 * 
 * https://www.mathworks.com/help/sltest/ref/sawtooth.html
 */
public class Sawtooth implements DoubleUnaryOperator {
    private final double a;
    private final double b;
    private final double p;

    /**
     * @param a amplitude
     * @param b offset
     * @param p period
     */
    public Sawtooth(double a, double b, double p) {
        this.a = a;
        this.b = b;
        this.p = p;
    }

    @Override
    public double applyAsDouble(double t) {
        double x = t / p;
        return a * (2 * (x - Math.floor(x)) - 1) + b;
    }
}
