// Minimal ESPHome log mock — no-ops that still "use" their arguments
// to avoid unused-variable warnings in production code under test.
#pragma once

namespace esphome {
namespace mock_log {
// Variadic template that silently consumes all arguments, preventing
// both unused-variable and comma-operator warnings.
template<typename... Args>
inline void noop(Args&&...) {}
}  // namespace mock_log
}  // namespace esphome

#define ESP_LOGE(tag, fmt, ...) esphome::mock_log::noop(tag, fmt __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) esphome::mock_log::noop(tag, fmt __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) esphome::mock_log::noop(tag, fmt __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) esphome::mock_log::noop(tag, fmt __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) esphome::mock_log::noop(tag, fmt __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGCONFIG(tag, fmt, ...) esphome::mock_log::noop(tag, fmt __VA_OPT__(,) __VA_ARGS__)
