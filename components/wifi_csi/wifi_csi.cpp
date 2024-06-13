#include "motionDetector.h"
#include <WiFi.h>
#include "esp_wifi.h"

#define ENABLE_ALARM_THRESHOLD 0
#define MAX_SAMPLEBUFFERSIZE 256
#define MAX_AVERAGEBUFFERSIZE 64
#define MAX_VARIANCE 65535
#define MINIMUM_RSSI -100

int enableThreshold = ENABLE_ALARM_THRESHOLD;
bool enableAutoRegressive = false;
int sampleBufferSize = MAX_SAMPLEBUFFERSIZE;
int mobileAverageFilterSize = MAX_SAMPLEBUFFERSIZE;
int mobileAverageBufferSize = MAX_AVERAGEBUFFERSIZE;
int varianceThreshold = 3;
int varianceIntegratorLimitMax = MAX_SAMPLEBUFFERSIZE;
int varianceIntegratorLimit = 3;
int varianceBufferSize = MAX_SAMPLEBUFFERSIZE;
int minimumRSSI = MINIMUM_RSSI;
int enableCSVout = 0;

int* sampleBuffer = NULL;
int sampleBufferIndex = 0;
int sampleBufferValid = 0;
int* mobileAverageBuffer = NULL;
int mobileAverageBufferIndex = 0;
int mobileAverageBufferValid = 0;
int* varianceBuffer = NULL;
int varianceBufferIndex = 0;
int varianceBufferValid = 0;
int mobileAverage = 0;
int mobileAverageTemp = 0;
int variance = RADAR_BOOTING;
int variancePrev = 0;
int varianceSample = 0;
int varianceAR = 0;
int varianceIntegral = 0;
int detectionLevel = 0;
int modeRes = 0;
int scanMode = SCANMODE_STA;
int strongestClientRSSI = -100;
int strongestClientfound = 0;
uint8_t strongestClientBSSID[6] = {0};
uint8_t BSSIDinUse[6] = {0};
int strongestRSSI = -100;
int strongestChannel = 0;
int strongestAPfound = 0;
uint8_t strongestBSSID[6] = {0};

int motionDetector_init() {
    if (sampleBuffer == NULL) {
        sampleBuffer = (int*)malloc(sizeof(int) * sampleBufferSize);
        for (int i = 0; i < sampleBufferSize; i++) {
            sampleBuffer[i] = 0x00;
        }
    }
    if (mobileAverageBuffer == NULL) {
        mobileAverageBuffer = (int*)malloc(sizeof(int) * mobileAverageBufferSize);
        for (int i = 0; i < mobileAverageBufferSize; i++) {
            mobileAverageBuffer[i] = 0x00;
        }
    }
    if (varianceBuffer == NULL) {
        varianceBuffer = (int*)malloc(sizeof(int) * varianceBufferSize);
        for (int i = 0; i < varianceBufferSize; i++) {
            varianceBuffer[i] = 0x00;
        }
    }
    variance = -1;
    return 1;
}

int motionDetector_init_PSRAM() {
    if (sampleBuffer == NULL) {
        sampleBuffer = (int*)ps_malloc(sizeof(int) * sampleBufferSize);
        for (int i = 0; i < sampleBufferSize; i++) {
            sampleBuffer[i] = 0x00;
        }
    }
    if (mobileAverageBuffer == NULL) {
        mobileAverageBuffer = (int*)ps_malloc(sizeof(int) * mobileAverageBufferSize);
        for (int i = 0; i < mobileAverageBufferSize; i++) {
            mobileAverageBuffer[i] = 0x00;
        }
    }
    if (varianceBuffer == NULL) {
        varianceBuffer = (int*)ps_malloc(sizeof(int) * varianceBufferSize);
        for (int i = 0; i < varianceBufferSize; i++) {
            varianceBuffer[i] = 0x00;
        }
    }
    variance = -1;
    return 1;
}

int motionDetector_deinit() {
    if (sampleBuffer != NULL) {
        free(sampleBuffer);
    }
    if (mobileAverageBuffer != NULL) {
        free(mobileAverageBuffer);
    }
    if (varianceBuffer != NULL) {
        free(varianceBuffer);
    }
    return 1;
}

int motionDetector_config(int sampleBufSize = 256, int mobileAvgSize = 64, int varThreshold = 3, int varIntegratorLimit = 3, bool enableAR = false) {
    if (sampleBufSize >= MAX_SAMPLEBUFFERSIZE) {
        sampleBufSize = MAX_SAMPLEBUFFERSIZE;
    }
    sampleBufferSize = sampleBufSize;

    varianceBufferSize = sampleBufferSize;

    if (mobileAvgSize >= MAX_SAMPLEBUFFERSIZE) {
        mobileAvgSize = MAX_SAMPLEBUFFERSIZE;
    }
    mobileAverageFilterSize = mobileAvgSize;

    if (varThreshold >= MAX_VARIANCE) {
        varThreshold = MAX_VARIANCE;
    }
    varianceThreshold = varThreshold;

    if (varIntegratorLimit >= varianceIntegratorLimitMax) {
        varIntegratorLimit = varianceIntegratorLimitMax;
    }
    varianceIntegratorLimit = varIntegratorLimit;

    enableAutoRegressive = enableAR;
    return 1;
}

int motionDetector_process(int sample = 0) {
    if ((sampleBuffer == NULL) || (mobileAverageBuffer == NULL) || (varianceBuffer == NULL)) {
        return RADAR_UNINITIALIZED;
    }

    if (sample < minimumRSSI) {
        return RADAR_RSSI_TOO_LOW;
    }

    sampleBuffer[sampleBufferIndex] = sample;
    sampleBufferIndex++;
    if (sampleBufferIndex >= sampleBufferSize) {
        sampleBufferIndex = 0;
        sampleBufferValid = 1;
    }

    if (sampleBufferValid >= 1) {
        mobileAverageTemp = 0;
        int mobilePointer = 0;
        for (int i = 0; i < mobileAverageFilterSize; i++) {
            mobilePointer = sampleBufferIndex - i;
            if (mobilePointer <= 0) {
                mobilePointer += (sampleBufferSize - 1);
            }
            mobileAverageTemp += sampleBuffer[mobilePointer];
        }
        mobileAverage = mobileAverageTemp / mobileAverageFilterSize;
        mobileAverageBuffer[mobileAverageBufferIndex] = mobileAverage;

        variancePrev = variance;
        varianceSample = (sample - mobileAverageBuffer[mobileAverageBufferIndex]) * (sample - mobileAverageBuffer[mobileAverageBufferIndex]);
        varianceBuffer[varianceBufferIndex] = varianceSample;

        varianceIntegral = 0;
        int variancePointer = 0;
        for (int i = 0; i < varianceIntegratorLimit; i++) {
            variancePointer = varianceBufferIndex - i;
            if (variancePointer <= 0) {
                variancePointer += (varianceBufferSize - 1);
            }
            varianceIntegral += varianceBuffer[variancePointer];
        }

        varianceBufferIndex++;
        if (varianceBufferIndex >= varianceBufferSize) {
            varianceBufferIndex = 0;
            varianceBufferValid = 1;
        }

        varianceAR = (varianceIntegral + varianceAR) / 2;

        variance = enableAutoRegressive ? varianceAR : varianceIntegral;

        mobileAverageBufferIndex++;
        if (mobileAverageBufferIndex >= mobileAverageBufferSize) {
            mobileAverageBufferIndex = 0;
            mobileAverageBufferValid = 1;
        }
    }

    if ((variance >= varianceThreshold) && (enableThreshold > 0)) {
        detectionLevel = variance;
        return detectionLevel;
    }

    if ((variance < varianceThreshold) && (variance >= 0) && (enableThreshold > 0)) {
        detectionLevel = 0;
        return detectionLevel;
    }

    return variance;
}

int motionDetector() {
    int RSSIlevel = (int)WiFi.RSSI();
    if (RSSIlevel == 0) {
        return RADAR_INOPERABLE;
    }

    return motionDetector_process(RSSIlevel);
}

int bistatic_get_rssi_SoftAP_strongestClient() {
    int rssi = 0;
    wifi_sta_list_t stationList;
    esp_err_t scanRes = esp_wifi_ap_get_sta_list(&stationList);

    if (scanRes != ESP_OK) {
        return 0;
    }

    if (strongestClientfound == 0) {
        strongestClientRSSI = -100;
        for (int i = 0; i < stationList.num; i++) {
            wifi_sta_info_t station = stationList.sta[i];
            int currentRSSI = station.rssi;
            if (currentRSSI > strongestClientRSSI) {
                strongestClientfound = 1;
                strongestClientRSSI = currentRSSI;
                memcpy(strongestClientBSSID, station.mac, 6);
            }
        }
    }

    int bssidScanOK = 0;
    if (strongestClientfound == 1) {
        for (int i = 0; i < stationList.num; i++) {
            wifi_sta_info_t station = stationList.sta[i];
            if (memcmp(station.mac, strongestClientBSSID, 6) == 0) {
                rssi = station.rssi;
                bssidScanOK = 1;
                break;
            }
        }
        if (bssidScanOK == 0) {
            strongestClientfound = 0;
        }
    }

    if (strongestClientfound == 0) {
        rssi = 0;
    }
    return rssi;
}
