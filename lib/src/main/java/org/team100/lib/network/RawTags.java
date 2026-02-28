package org.team100.lib.network;

import java.util.function.ObjDoubleConsumer;

import org.team100.lib.config.Camera;
import org.team100.lib.localization.Blip;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.StructBuffer;

/** Listen to raw tag input from the cameras, for testing. */
public class RawTags extends CameraReader<Blip> {
    private final ObjDoubleConsumer<Transform3d> m_sink;

    public RawTags(LoggerFactory parent, ObjDoubleConsumer<Transform3d> sink) {
        super(parent, "vision", "blips", StructBuffer.create(Blip.struct));
        m_sink = sink;
    }

    @Override
    protected void perValue(Camera camera, Blip[] value) {
        for (Blip b : value) {
            m_sink.accept(b.blipToTransform(), (double) b.getTimestamp() / 1e6);
        }
    }

}
