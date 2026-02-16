package org.team100.lib.util.wave;

import java.util.function.DoubleSupplier;

/**
 * A sawtooth wave.
 * 
 * https://www.mathworks.com/help/sltest/ref/sawtooth.html
 */
public class Sawtooth implements DoubleSupplier {
    private final DoubleSupplier f;
    private final double a;
    private final double b;
    private final double p;

    /**
     * @param f time
     * @param a amplitude
     * @param b offset
     * @param p period
     */
    public Sawtooth(DoubleSupplier f, double a, double b, double p) {
        this.f = f;
        this.a = a;
        this.b = b;
        this.p = p;
    }

    @Override
    public double getAsDouble() {
        double t = f.getAsDouble();
        double x = t / p;
        return a * (2 * (x - Math.floor(x)) - 1) + b;
    }
}
