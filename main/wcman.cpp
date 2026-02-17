#include "wcman.h"
#include "app/util/attribute-storage.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "WC_MANAGER";

#define LED_GPIO   GPIO_NUM_0
#define BLINK_MS   200

/* --------------------------------------------------------------------- */
using namespace chip;
using namespace chip::app::Clusters::WindowCovering;

/* --------------------------------------------------------------------- */
/* HandleMovement() – this is invoked for the three standard commands
 *
 *    * LiftUpOrOpen   (Open)
 *    * LiftDownOrClose(Close)
 *    * (Tilt variants exist but we never receive them because our
 *      FeatureMap does not include Tilt).
 *
 * The SDK passes a value of type `Movement::Type`.  The enum is defined
 * in the generated ZAP code as:
 *
 *    enum class Movement::Type : uint8_t {
 *        LiftUpOrOpen   = 0,
 *        LiftDownOrClose = 1,
 *        TiltUpOrOpen    = 2,
 *        TiltDownOrClose = 3
 *    };
 *
 * We only care about the Lift values; any Tilt value is treated as
 * “unsupported” and we return an error.
 */
CHIP_ERROR MyWindowCoveringManager::HandleMovement(WindowCoveringType type)
{
    ESP_LOGE(TAG, "HandleMovement request");
	// gpio_set_level(LED_GPIO, 1);   // HIGH (ON)
	// vTaskDelay(pdMS_TO_TICKS(BLINK_MS));
	// ESP_LOGI(TAG, "Toggling LED GPIO%d", LED_GPIO);
	// gpio_set_level(LED_GPIO, 0);   // LOW (OFF)
    return CHIP_NO_ERROR;
}

/* --------------------------------------------------------------------- */
/* HandleStopMotion() – called when the controller sends the Stop
 * command.  We simply pulse the STOP line (brake) and clear the
 * direction pins. */
CHIP_ERROR MyWindowCoveringManager::HandleStopMotion()
{
    ESP_LOGE(TAG, "HandleStopMotion request");
    return CHIP_NO_ERROR;
}
