#include "wifi_csi.h"
#include <cmath>
#include <deque>
#include <algorithm>

static const char *const TAG = "wifi_csi";
extern esphome::wifi::WiFiComponent *esphome::wifi::global_wifi_component;

// Constants for the motion detection logic
constexpr float ALPHA = 0.3;                  // Smoothing factor for EWMA
constexpr float THRESHOLD_MULTIPLIER = 1.5;   // Multiplier for adjusting the threshold
constexpr int LOG_INTERVAL = 5;               // Logging interval in seconds
constexpr int MIN_STABLE_TIME = 10;           // Minimum time in seconds to consider RSSI stable
constexpr float SENSITIVITY_MULTIPLIER = 1.8; // Multiplier for sensitivity adjustment

// Kalman filter parameters
constexpr float Q = 0.01; // Process noise covariance
constexpr float R = 0.5;  // Measurement noise covariance

esphome::wifi_csi::CsiSensor::CsiSensor()
: PollingComponent(), binary_sensor::BinarySensor(),
  m_pollingInterval(100), m_bufferSize(100), m_sensitivity(1.5), m_rssi(nullptr),
  m_kalman_x(0), m_kalman_p(1)
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
    static bool last_state = false;
    static std::deque<float> recent_stdvs;

    if (m_rssi) {
        int currentRssi = esphome::wifi::global_wifi_component ? esphome::wifi::global_wifi_component->wifi_rssi() : 0;

        // Apply Kalman filter to the current RSSI value
        float filteredRssi = apply_kalman_filter(currentRssi);

        if (cnt == m_bufferSize) {
            update_rssi_buffer(filteredRssi, idx, cnt, sum, stdv);
        } else {
            m_rssi[idx] = filteredRssi;
            sum += filteredRssi;
            ++cnt;
        }

        if (cnt == m_bufferSize) {
            float avgRssi = sum / cnt;
            float diff = pow((filteredRssi - avgRssi), 2);
            stdv += diff;

            if (idx == m_bufferSize - 1) {
                stdv = sqrt(stdv / m_bufferSize);
                ewma_stdv = ALPHA * stdv + (1 - ALPHA) * ewma_stdv;

                // Update threshold dynamically based on recent stdv history
                recent_stdvs.push_back(ewma_stdv);
                if (recent_stdvs.size() > MIN_STABLE_TIME) {
                    recent_stdvs.pop_front();
                }
                float median_stdv = calculate_median(recent_stdvs);
                threshold = median_stdv * THRESHOLD_MULTIPLIER;
                new_state = (stdv - m_sensitivity) > threshold;

                // Apply hysteresis
                if (new_state != last_state) {
                    if (new_state) {
                        if ((stdv - m_sensitivity) > (threshold + m_sensitivity * SENSITIVITY_MULTIPLIER)) {
                            last_state = new_state;
                        }
                    } else {
                        if ((stdv - m_sensitivity) < (threshold - m_sensitivity * SENSITIVITY_MULTIPLIER)) {
                            last_state = new_state;
                        }
                    }
                }

                ESP_LOGD(TAG, "stdv: %.2f, ewma_stdv: %.2f, threshold: %.2f, state: %d", stdv, ewma_stdv, threshold, last_state);
                stdv = 0;
            }
        }

        idx = (idx + 1) % m_bufferSize;
        log_rssi_data(idx, cnt, sum, filteredRssi, last_state);
        this->publish_state(last_state);
    } else {
        set_buffer_size(m_bufferSize);
    }
}

void esphome::wifi_csi::CsiSensor::update_rssi_buffer(float currentRssi, int& idx, int& cnt, float& sum, float& stdv)
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

void esphome::wifi_csi::CsiSensor::log_rssi_data(int idx, int cnt, float sum, float currentRssi, bool new_state)
{
    static time_t last_t;
    time_t now_t;
    time(&now_t);

    if (difftime(now_t, last_t) > LOG_INTERVAL) {
        float avgRssi = sum / cnt;
        ESP_LOGD(TAG, "idx: %d, cnt: %d, avg: %.1f, current: %.1f, sensitivity: %.2f, state: %d", idx, cnt, avgRssi, currentRssi, m_sensitivity, new_state);
        last_t = now_t;
    }
}

float esphome::wifi_csi::CsiSensor::calculate_median(std::deque<float>& data) {
    std::vector<float> sorted_data(data.begin(), data.end());
    std::sort(sorted_data.begin(), sorted_data.end());
    size_t mid = sorted_data.size() / 2;
    return sorted_data.size() % 2 == 0 ? (sorted_data[mid - 1] + sorted_data[mid]) / 2 : sorted_data[mid];
}

float esphome::wifi_csi::CsiSensor::apply_kalman_filter(float measurement) {
    // Predict
    float x_pred = m_kalman_x;
    float p_pred = m_kalman_p + Q;

    // Update
    float k = p_pred / (p_pred + R);
    m_kalman_x = x_pred + k * (measurement - x_pred);
    m_kalman_p = (1 - k) * p_pred;

    return m_kalman_x;
}
