package org.team100.lib.subsystems.five_bar.setups;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.five_bar.FiveBarBare;

import edu.wpi.first.wpilibj.XboxController;

public class SetupBare implements Runnable {
    private final FiveBarBare m_fiveBar;

    public SetupBare() {
        Logging logging = Logging.instance();
        LoggerFactory logger = logging.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        XboxController controller = new XboxController(0);
        m_fiveBar = new FiveBarBare(logger, currentLog);
        m_fiveBar.setDefaultCommand(m_fiveBar.dutyCycle(
                controller::getLeftX, controller::getRightX));
    }

    @Override
    public void run() {
        //
    }
}
