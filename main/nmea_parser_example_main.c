/* NMEA Parser example, that decode data stream from GPS receiver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nmea_parser.h"
#include "driver/twai.h"

#define TX_GPIO_NUM             CONFIG_EXAMPLE_TX_GPIO_NUM
#define RX_GPIO_NUM             CONFIG_EXAMPLE_RX_GPIO_NUM
#define TX_TASK_PRIO            8       //Sending task priority
#define CTRL_TSK_PRIO           10      //Control task priority
#define MSG_ID                  0x3   //11 bit standard format ID

static const char *TAG = "gps_demo";

#define TIME_ZONE (+8)   //Beijing Time
#define YEAR_BASE (2000) //date in GPS starts from 2000

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//Set to NO_ACK mode due to self testing with single module
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NO_ACK);
static const twai_filter_config_t f_config = {
    .acceptance_code = 0,
    .acceptance_mask = 0xFFFFFFFF,
    .single_filter = true
};
twai_message_t tx_msg = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 0,                // Not single shot
    .self = 1,              // Message is a self reception request (loopback)
    .dlc_non_comp = 0,      // DLC is less than 8
    // Message ID and payload
    .identifier = MSG_ID,
    .data_length_code = 8,
    .data = {0,0,0,0,0,0,0,0},
};

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gps_t *gps = NULL;
    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements */
        ESP_LOGI(TAG, "Raw NMEA sentence:%s", gps->raw_data);
        ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
                       
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

void app_main(void)
{
    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, " CAN Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN Driver started");
    /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* init NMEA parser library */
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

    while (1) {
        ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY)); //Send message
        vTaskDelay(10 / portTICK_PERIOD_MS); // 每10ms延迟一次
    }

    /* unregister event handler */
    nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
    /* deinit NMEA parser library */
    nmea_parser_deinit(nmea_hdl);
}
