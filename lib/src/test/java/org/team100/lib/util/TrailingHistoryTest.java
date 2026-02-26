package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Test;

public class TrailingHistoryTest {
    /** Verify presence but not order */
    void check(TrailingHistory<String> h, String... expected) {
        List<String> actual = h.getAll();
        List<String> sortedActual = new ArrayList<>(actual);
        Collections.sort(sortedActual);
        List<String> sortedExpected = Arrays.asList(expected);
        Collections.sort(sortedExpected);
        assertEquals(sortedExpected, sortedActual);
    }

    @Test
    void testSimple() {
        TrailingHistory<String> h = new TrailingHistory<>();
        h.evict(-1);
        h.add(0, "zero");
        check(h, "zero");
    }

    @Test
    void testEviction() {
        TrailingHistory<String> h = new TrailingHistory<>();
        h.evict(-1);
        h.add(0, "zero");
        // should evict 0.
        h.evict(1);
        h.add(2, "two");
        check(h, "two");
    }

    @Test
    void testNonEviction() {
        TrailingHistory<String> h = new TrailingHistory<>();
        h.evict(-1);
        h.add(0, "zero");
        // should not evict 0.
        h.evict(-0.5);
        h.add(0.5, "one half");
        check(h, "zero", "one half");
    }

    @Test
    void testOutOfOrder() {
        TrailingHistory<String> h = new TrailingHistory<>();
        h.evict(1);
        h.add(2, "two");
        // should not evict anything
        h.evict(-1);
        h.add(0, "zero");
        check(h, "two", "zero");
    }
}
