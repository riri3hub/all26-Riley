package org.team100.lib.util;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A collection with time-based eviction.
 * 
 * Eviction is separate from addition, because many times we have nothing to
 * add.
 */
public class TrailingHistory<T> {
    public record ValueRecord<T>(double time, T value) {
    };

    private final List<ValueRecord<T>> m_entries;

    public TrailingHistory() {
        m_entries = new ArrayList<ValueRecord<T>>();
    }

    /** Add the new value. */
    public void add(double time, T value) {
        m_entries.add(new ValueRecord<>(time, value));
    }

    public List<T> getAll() {
        return m_entries.stream()
                .map((x) -> x.value)
                .collect(Collectors.toUnmodifiableList());
    }

    /** Mutating iterator for filtering. */
    public Iterator<ValueRecord<T>> iterator() {
        return m_entries.iterator();
    }

    public int size() {
        return m_entries.size();
    }

    /**
     * Evict entries whose timestamps are earlier than the deadline.
     * 
     * This could take the current time and apply the delta, but it seems simpler to
     * let the caller figure out what deadline to use.
     * 
     * @param deadline seconds
     */
    public void evict(double deadline) {
        m_entries.removeIf(x -> x.time < deadline);
    }

}
