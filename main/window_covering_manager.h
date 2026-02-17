// #pragma once

// #include <app/clusters/window-covering-server/window-covering-server.h>

// namespace chip {
// namespace app {
// namespace Clusters {
// namespace WindowCovering {

// /**
//  * Our concrete delegate – we implement the *command‑specific* callbacks.
//  * The generic HandleMovement/HandleStopMotion are no longer required.
//  */
// class WindowCoveringManager : public Delegate
// {
// public:
//     void Init(chip::EndpointId endpoint);

//     // ---- Fine‑grained callbacks (required) ----
//     CHIP_ERROR HandleUpOrOpen(chip::EndpointId endpointId) override;
//     CHIP_ERROR HandleDownOrClose(chip::EndpointId endpointId) override;
//     CHIP_ERROR HandleStopMotion(chip::EndpointId endpointId) override;

//     // Optional – we ignore the percentage because we have no encoder
//     CHIP_ERROR HandleGoToLiftPercentage(chip::EndpointId endpointId,
//                                         uint8_t liftPercent) override
//     {
//         // Treat it exactly like “UpOrOpen” – start moving up.
//         ESP_LOGI(TAG, "GoToLiftPercentage %u → start opening (ignore exact %)", liftPercent);
//         motor_start_up();
//         report_operational_status(g_wc_endpoint,
//                                   OperationalStatusEnum::kOpening);
//         return CHIP_NO_ERROR;
//     }

//     // ---- Tilt callbacks – not supported on this device ----
//     CHIP_ERROR HandleGoToTiltPercentage(chip::EndpointId, uint8_t) override
//     {
//         return CHIP_ERROR_NOT_IMPLEMENTED;
//     }
//     CHIP_ERROR HandleStopTilt(chip::EndpointId) override
//     {
//         return CHIP_ERROR_NOT_IMPLEMENTED;
//     }

// private:
//     // You can keep any state you want (current lift position, etc.)
//     // For a pure lift‑only device we usually do not need extra state.
// };

// } // namespace WindowCovering
// } // namespace Clusters
// } // namespace app
// } // namespace chip