package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.DiscreteFunction.Point;

import edu.wpi.first.math.interpolation.Interpolator;

public class DiscreteFunctionTest {
    private static final boolean DEBUG = true;
    private static final double DELTA = 1e-12;

    @Test
    void testMonotonic() {
        DiscreteFunction<String> f = new DiscreteFunction<>();
        f.put(0.0, 0.0, "a");
        f.put(1.0, 10.0, "b");
        f.put(2.0, 20.0, "c");
        if (DEBUG)
            System.out.println("F");
        dump(f);
        DiscreteFunction<String> finv = f.inverse();
        if (DEBUG)
            System.out.println("Finv");
        dump(finv);
    }

    @Test
    void testNonMonotonic() {
        DiscreteFunction<String> f = new DiscreteFunction<>();
        f.put(0.0, 0.0, "a");
        f.put(1.0, 10.0, "b");
        f.put(2.0, 0.0, "c");
        if (DEBUG)
            System.out.println("F");
        dump(f);
        assertThrows(IllegalStateException.class, f::inverse);
    }

    @Test
    void testZeroDerivative() {
        DiscreteFunction<String> f = new DiscreteFunction<>();
        f.put(0.0, 0.0, "a");
        f.put(1.0, 10.0, "b");
        f.put(2.0, 10.0, "c");
        if (DEBUG)
            System.out.println("F");
        dump(f);
        assertThrows(IllegalStateException.class, f::inverse);
    }

    @Test
    void testMap() {
        DiscreteFunction<String> f = new DiscreteFunction<>();
        f.put(0.0, 0.0, "a");
        f.put(0.5, 5.0, "b");
        f.put(1.0, 10.0, "c");
        InterpolatingMap100<Double, Point<String>> m = f.map(
                new Interpolator<String>() {
                    @Override
                    public String interpolate(String startValue, String endValue, double t) {
                        return startValue;
                    }
                });
        assertNull(m.get(-1.0));
        assertEquals(0.0, m.get(0.0).y(), DELTA);
        assertEquals(2.5, m.get(0.25).y(), DELTA);
        assertEquals(5.0, m.get(0.5).y(), DELTA);
        assertEquals(7.5, m.get(0.75).y(), DELTA);
        assertEquals(10.0, m.get(1.0).y(), DELTA);
        assertNull(m.get(2.0));
    }

    @Test
    void testInvMap() {
        DiscreteFunction<String> f = new DiscreteFunction<>();
        f.put(0.0, 0.0, "a");
        f.put(0.5, 5.0, "b");
        f.put(1.0, 10.0, "c");
        InterpolatingMap100<Double, Point<String>> m = f.inverse().map(
                new Interpolator<String>() {
                    @Override
                    public String interpolate(String startValue, String endValue, double t) {
                        return startValue;
                    }
                });
        assertNull(m.get(-10.0));
        assertEquals(0.0, m.get(0.0).y(), DELTA);
        assertEquals(0.25, m.get(2.5).y(), DELTA);
        assertEquals(0.5, m.get(5.0).y(), DELTA);
        assertEquals(0.75, m.get(7.5).y(), DELTA);
        assertEquals(1.0, m.get(10.0).y(), DELTA);
        assertNull(m.get(20.0));
    }

    private <V> void dump(DiscreteFunction<V> f) {
        for (Point<V> p : f.points()) {
            if (DEBUG)
                System.out.printf("%s\n", p);
        }
    }

}
