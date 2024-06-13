#include "wifi_csi.h"
#include <cmath>

static const char *const TAG = "wifi_csi";
extern esphome::wifi::WiFiComponent *esphome::wifi::global_wifi_component;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

constexpr float ALPHA = 0.2;
constexpr float THRESHOLD_MULTIPLIER = 1.2;
constexpr int LOG_INTERVAL = 5;

esphome::wifi_csi::CsiSensor::CsiSensor()
: PollingComponent(), binary_sensor::BinarySensor(),
  m_pollingInterval(100), m_bufferSize(100), m_sensitivity(2), m_rssi(nullptr)
{
    set_update_interval(m_pollingInterval);
    this->set_device_class("motion");
}

esphome::wifi_csi::CsiSensor::~CsiSensor()
{
    if (m_rssi) {
        ESP_LOGD(TAG, "Freeing RSSI buffer");
        free(m_rssi);
        m_rssi = nullptr;
    }
}

float esphome::wifi_csi::CsiSensor::get_setup_priority() const
{
    return esphome::setup_priority::AFTER_WIFI;
}

void esphome::wifi_csi::CsiSensor::dump_config()
{
    ESP_LOGCONFIG(TAG, "Wifi CSI:");
    ESP_LOGCONFIG(TAG, "Polling interval: %dms", m_pollingInterval);
    ESP_LOGCONFIG(TAG, "Buffer size: %d", m_bufferSize);
    ESP_LOGCONFIG(TAG, "Sensitivity: %.2f", m_sensitivity);
}

void esphome::wifi_csi::CsiSensor::set_timing(int pollingInterval)
{
    m_pollingInterval = pollingInterval;
    set_update_interval(pollingInterval);
}

void esphome::wifi_csi::CsiSensor::set_sensitivity(float sensitivity)
{
    m_sensitivity = sensitivity;
}

void esphome::wifi_csi::CsiSensor::set_buffer_size(int bufferSize)
{
    m_bufferSize = bufferSize;
    if (m_rssi != nullptr) free(m_rssi);
    m_rssi = reinterpret_cast<int*>(malloc(m_bufferSize * sizeof(int)));
}

void esphome::wifi_csi::CsiSensor::update() {
    static int idx = 0;
    static int cnt = 0;
    static float sum = 0.0;
    static float stdv = 0;
    static float ewma_stdv = 0;
    static float threshold = 1.3;
    static bool new_state = false;
    static bool last_state = false; // To handle hysteresis

    if (m_rssi) {
        int currentRssi = esphome::wifi::global_wifi_component ? esphome::wifi::global_wifi_component->wifi_rssi() : 0;

        if (cnt == m_bufferSize) {
            update_rssi_buffer(currentRssi, idx, cnt, sum, stdv);
        } else {
            m_rssi[idx] = currentRssi;
            sum += currentRssi;
            ++cnt;
        }

        if (cnt == m_bufferSize) {
            float avgRssi = sum / cnt;
            float diff = pow((currentRssi - avgRssi), 2);
            stdv += diff;

            if (idx == m_bufferSize - 1) {
                stdv = sqrt(stdv / m_bufferSize);
                ewma_stdv = ALPHA * stdv + (1 - ALPHA) * ewma_stdv;
                threshold = ewma_stdv * THRESHOLD_MULTIPLIER;
                new_state = (stdv - m_sensitivity) > threshold;

                // Apply hysteresis
                if (new_state != last_state) {
                    if (new_state) {
                        if ((stdv - m_sensitivity) > (threshold + m_sensitivity)) {
                            last_state = new_state;
                        }
                    } else {
                        if ((stdv - m_sensitivity) < (threshold - m_sensitivity)) {
                            last_state = new_state;
                        }
                    }
                }

                ESP_LOGD(TAG, "stdv: %.2f, ewma_stdv: %.2f, threshold: %.2f, state: %d", stdv, ewma_stdv, threshold, last_state);
                stdv = 0;
            }
        }

        idx = (idx + 1) % m_bufferSize;
        log_rssi_data(idx, cnt, sum, currentRssi, last_state);
        this->publish_state(last_state);
    } else {
        set_buffer_size(m_bufferSize);
    }
}

void esphome::wifi_csi::CsiSensor::update_rssi_buffer(int currentRssi, int& idx, int& cnt, float& sum, float& stdv)
{
    sum -= m_rssi[idx];
    m_rssi[idx] = currentRssi;
    sum += currentRssi;

    if (cnt == m_bufferSize) {
        float avgRssi = sum / cnt;
        float diff = pow((currentRssi - avgRssi), 2);
        stdv += diff;
    }
}

void esphome::wifi_csi::CsiSensor::log_rssi_data(int idx, int cnt, float sum, int currentRssi, bool new_state)
{
    static time_t last_t;
    time_t now_t;
    time(&now_t);

    if (difftime(now_t, last_t) > LOG_INTERVAL) {
        float avgRssi = sum / cnt;
        ESP_LOGD(TAG, "idx: %d, cnt: %d, avg: %.1f, current: %d, sensitivity: %.2f, state: %d", idx, cnt, avgRssi, currentRssi, m_sensitivity, new_state);
        last_t = now_t;
    }
}
