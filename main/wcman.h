/* wc_manager.h ------------------------------------------------------------ */
#pragma once

#include <app/clusters/window-covering-server/window-covering-server.h>
#include <app/clusters/window-covering-server/window-covering-delegate.h>

namespace chip {
namespace app {
namespace Clusters {
namespace WindowCovering {

/**
 * Concrete implementation of the WindowCovering::Delegate that talks to the
 * ESP‑32 motor driver.
 *
 * Only the **Lift** feature is supported (no Tilt).  The class publishes the
 * standard MotionStarted / MotionStopped events so that a Matter controller
 * can see what is happening.
 */
class MyWindowCoveringManager: public Delegate
{
public:
    CHIP_ERROR HandleMovement(WindowCoveringType type) override;
    CHIP_ERROR HandleStopMotion() override;
};

} // namespace WindowCovering
} // namespace Clusters
} // namespace app
} // namespace chip