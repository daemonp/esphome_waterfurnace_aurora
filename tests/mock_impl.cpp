// Implementations for ESPHome mocks that need storage
#include "esphome/core/component.h"

namespace esphome {

static uint32_t s_mock_millis = 0;
uint32_t &mock_millis_ref() { return s_mock_millis; }

}  // namespace esphome
