#include "wifi_csi.h"
#include <cmath>
#include <deque>
#include <algorithm>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TXD_PIN GPIO_NUM_17   // TX pin
#define RXD_PIN GPIO_NUM_18   // RX pin

void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0));
}

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
init_uart();

uint8_t data[128];
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

                while (1) {
                    int len = uart_read_bytes(UART_NUM_1, data, sizeof(data) - 1, 1000 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "uart_read_bytes returned: %d", len);

                    if (len > 0) {
                        data[len] = '\0';  // Null-terminate the string
                        ESP_LOGI(TAG, "Received: '%s'", data);
                        this->publish_state(data);

                    } else {
                        ESP_LOGI(TAG, "No data received");
                    }
                    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
                }
                ESP_LOGD(TAG, "stdv: %.2f, ewma_stdv: %.2f, threshold: %.2f, state: %d", stdv, ewma_stdv, threshold, last_state);

                stdv = 0;
            }
        }

        idx = (idx + 1) % m_bufferSize;
        log_rssi_data(idx, cnt, sum, filteredRssi, last_state);
        // this->publish_state(last_state);

        // 

        this->publish_state(last_state);

    } else {
        set_buffer_size(m_bufferSize);
    }

           // stdv_part += diff;
            // if ((idx + 1) % 20 == 0){       // stdv each 20 rssi waves
            //     stdv_part = sqrt(stdv_part / 20);
            //     // ESP_LOGD(TAG,"stdv: %.2f",stdv);
            //     ESP_LOGD(TAG,"stdv each 20: %.2f",stdv_part);
            //     // if (stdv_part > 1.0){
            //     //     publish_state(true);
            //     //     ESP_LOGD(TAG,"published ON from stdv20 ");
            //     // }
            //     // publish_state(stdv_part > 1.3);
            //     stdv_part = 0;

            // }

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
