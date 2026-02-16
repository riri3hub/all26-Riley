package org.team100.lib.util.wave;

import java.util.function.DoubleSupplier;

/** A stair step proxy. */
public class Stair implements DoubleSupplier {
    private final DoubleSupplier f;
    private final double a;

    /**
     * @param f time
     * @param a scale
     */
    public Stair(DoubleSupplier f, double a) {
        this.f = f;
        this.a = a;
    }

    @Override
    public double getAsDouble() {
        double t = f.getAsDouble();
        return Math.ceil(a * t);
    }
}
