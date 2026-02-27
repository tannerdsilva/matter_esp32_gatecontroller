#include "bindings_core.h"
#include <esp_log.h>
#include <app/clusters/bindings/binding-table.h>

void handle_binding_changed_event() {
	ESP_LOGI("BIND", "BINDING TABLE CHANGED");
	for (auto &binding : chip::app::Clusters::Binding::Table::GetInstance()) {

	}
}