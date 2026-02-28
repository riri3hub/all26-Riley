package org.team100.lib.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.function.BiPredicate;
import java.util.function.Function;

import org.team100.lib.util.TrailingHistory.ValueRecord;

/**
 * A collection that combines new entries with old ones if they're close enough.
 * 
 * This is brute-force, iterating through the whole collection on every write,
 * so don't let it get too large.
 */
public class CoalescingCollection<T> {
    private static final double HISTORY_DURATION = 1.0;

    private final TrailingHistory<T> m_delegate;
    /** True if items should be combined. */
    private final BiPredicate<T, T> m_near;
    private final Function<Collection<T>, T> m_combine;

    public CoalescingCollection(
            TrailingHistory<T> delegate,
            BiPredicate<T, T> near,
            Function<Collection<T>, T> combine) {
        m_delegate = delegate;
        m_near = near;
        m_combine = combine;
    }

    /**
     * @param time seconds
     */
    public void add(double time, T value) {
        List<T> neighbors = new ArrayList<>();
        neighbors.add(value);
        Iterator<ValueRecord<T>> iter = m_delegate.iterator();
        while (iter.hasNext()) {
            ValueRecord<T> vr = iter.next();
            T v = vr.value();
            if (m_near.test(v, value)) {
                iter.remove();
                neighbors.add(v);
            }
        }
        T rep = m_combine.apply(neighbors);
        m_delegate.evict(time - HISTORY_DURATION);
        m_delegate.add(time, rep);
    }

    public void addAll(double time, Collection<T> values) {
        // this does unnecesary extra work
        values.forEach(v -> add(time, v));
    }

    public List<T> getAll() {
        return m_delegate.getAll();
    }

    public int size() {
        return m_delegate.size();
    }

}
