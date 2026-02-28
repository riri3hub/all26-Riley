package org.team100.lib.util.wave;

import java.util.function.DoubleUnaryOperator;

/** A stair step. */
public class Stair implements DoubleUnaryOperator {
    private final double a;

    /**
     * @param a scale
     */
    public Stair(double a) {
        this.a = a;
    }

    @Override
    public double applyAsDouble(double t) {
        return Math.ceil(a * t);
    }
}
