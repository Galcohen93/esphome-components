/**
 * @brief Simple motion detection sensor that uses the WiFi signal strength
 *        signal (RSSI) to detect motions.
 *
 * @author Jan Peter Riegel <JanPeter1@familie-riegel.de>
 * Copyright (c) 2022
 */

#include "wifi_csi.h"



static const char *const TAG = "wifi_csi";
extern esphome::wifi::WiFiComponent *esphome::wifi::global_wifi_component;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

esphome::wifi_csi::CsiSensor::CsiSensor()
: PollingComponent()
, binary_sensor::BinarySensor()
, m_pollingInterval(100)
, m_bufferSize(100)
, m_sensitivity(2)
, m_rssi(nullptr)
{
    set_update_interval(m_pollingInterval);
    this->set_device_class("motion");
}

esphome::wifi_csi::CsiSensor::~CsiSensor()
{
    if (m_rssi) {
        ESP_LOGD(TAG, "rssi");
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
    ESP_LOGCONFIG(TAG, "polling interval: %dms", m_pollingInterval);
    ESP_LOGCONFIG(TAG, "buffer size: %d", m_bufferSize);
    ESP_LOGCONFIG(TAG, "sensitivity: %.2f", m_sensitivity);
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
    static int idx = 0;   // pointer inside rssi
    static int cnt = 0;   // number of values inside rssi
    static float sum = 0.0;   // sum of all rssi values
    static float stdv = 0; // stdv 
    static float stdv_part = 0; // stdv of 20 rssi
    static float ewma_stdv = 0; // EWMA of stdv
    static float alpha = 0.3; // Smoothing factor for EWMA
    static float threshold = 1.3; // Initial threshold value

    if (m_rssi) {        
        float avgerageRssi = 0;    
        int currentRssi = 0;
        bool motion = 0;
        if (nullptr != esphome::wifi::global_wifi_component) currentRssi = esphome::wifi::global_wifi_component->wifi_rssi();

        if (cnt == m_bufferSize) {
            sum -= m_rssi[idx];  // we will overwrite the oldest value, so remove it from the current sum

            avgerageRssi = sum / cnt;
            float diff = pow((currentRssi - avgerageRssi),2);
            stdv += diff;
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

            if (idx == m_bufferSize - 1){
                stdv = sqrt(stdv / m_bufferSize);
                ESP_LOGD(TAG,"stdv: %.2f",stdv);

                // Update EWMA
                ewma_stdv = alpha * stdv + (1 - alpha) * ewma_stdv;

                // Adjust the threshold based on EWMA
                threshold = ewma_stdv * 1.3; // Adjust the multiplier as needed

                // Publish state based on adjusted threshold
                bool new_state = stdv > threshold;
                publish_state(new_state);

                ESP_LOGD(TAG, "ewma_stdv: %.2f, threshold: %.2f, state: %d", ewma_stdv, threshold, new_state);
                // ESP_LOGD(TAG,"stdv: %.2f, curRssi: %d , avgRssi: %.2f, motion: %d",stdv,currentRssi,avgerageRssi,motion);
                stdv = 0;
            }
            // float dev = abs(m_rssi[idx] - avgerageRssi);
            // motion = (dev >= m_sensitivity);

            // publish_state(motion);

        } else {
            cnt += 1;

        }
        m_rssi[idx] = currentRssi;
        idx = (idx + 1) % m_bufferSize;
        sum += currentRssi;
        // ESP_LOGD(TAG,"rssi idx: %d",m_rssi[idx]);


        // log every 5 seconds
        static time_t last_t;
        time_t now_t;
        time(&now_t);
        if (difftime(now_t, last_t) > 5.0) {
            ESP_LOGD(TAG, "idx: %d, cnt: %d: avg: %.1f, current: %d, sensitvity: %.2f, motion: %d", idx, cnt, avgerageRssi, currentRssi, m_sensitivity, motion);
            last_t = now_t;
        }
    } 
    else {
        set_buffer_size(m_bufferSize);
    }
}


