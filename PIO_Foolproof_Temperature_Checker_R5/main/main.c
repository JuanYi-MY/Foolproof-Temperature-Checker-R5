/*
 * Foolproof_Temperature_Checker_R5 (based on AWS IoT EduKit - Core2 for AWS IoT EduKit)
 * main.c
 * 
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/**
 * @file main.c
 * @brief simple MQTT publish, subscribe, and device shadows for use with AWS IoT EduKit reference hardware.
 *
 * This example takes the parameters from the build configuration and establishes a connection to AWS IoT Core over MQTT.
 *
 * Some configuration is required. Visit https://edukit.workshop.aws
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_shadow_interface.h"

#include "core2forAWS.h"

#include "wifi.h"
#include "fft.h"
#include "ui.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "MAIN";

#define MAX_LENGTH_OF_UPDATE_JSON_BUFFER 500

/* CA Root certificate */
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");

/* Default MQTT HOST URL is pulled from the aws_iot_config.h */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/* Default MQTT port is pulled from the aws_iot_config.h */
uint32_t port = AWS_IOT_MQTT_PORT;

TaskHandle_t led_bar_blink_handle;

static void touch_task(void *pvParameters);

bool startUp = true; // flag to run 1st connection upon startup
char temp_str[200];
bool testDesired_prev = false;

char temp_str_awsThres[200];
char temp_str_awsCount[200];

// *** json variable for mqtt ***********************************************************************
bool testReported_Increase = false;
bool testReported_Decrease = false;
bool testDesired = false;
// uint16_t threshold_fr_aws = 0;
uint16_t thres_fr_aws = 0;
uint16_t count_fr_aws = 0;

// *** uart with openmv **************************************************************************
static void uart_tx_task(void *arg){
    char enableFlag_to_omv[1];
    while (1) {
        if (startUp == false && testDesired_prev != testDesired) {
            if (testDesired == true ) {
                enableFlag_to_omv[0] = '1';  // to enable openmv, blue led openmv will on    
            } else {
                enableFlag_to_omv[0] = '2';  // to disable openmv, blue led openmv will off
            }
            // size_t message_len = strlen(enableFlag_to_omv);
            Core2ForAWS_Port_C_UART_Send(enableFlag_to_omv, 1);
            ESP_LOGI(TAG, "UART send\t: %s", enableFlag_to_omv);
            testDesired_prev = testDesired;
        }
        //ESP_LOGI(TAG, "testDesired\t: %d", testDesired);
        //ESP_LOGI(TAG, "testDesired_prev\t: %d", testDesired_prev);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
 
void uart_rx_logic(char status_fr_omv, float bodyTemp_fr_omv){
    if (status_fr_omv == '1') {  // set LED blue blink & text "Measuring"
        vTaskResume(led_bar_blink_handle);
        ESP_LOGI(TAG, "Status is 1.");
        sprintf(temp_str, "MEASURING...");     
        ui_textlabel_add(temp_str);

    } else if (status_fr_omv == '2') { // set LED green or red & text according to result
        vTaskSuspend(led_bar_blink_handle);
        ESP_LOGI(TAG, "Status is 2.");
        if (bodyTemp_fr_omv >=36.0 && bodyTemp_fr_omv < 37.5){  // Set green and IOT
            Core2ForAWS_Sk6812_SetBrightness(40);
            Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_LEFT, 0x00b050);
            Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_RIGHT, 0x00b050);
            Core2ForAWS_Sk6812_Show();
            
            sprintf(temp_str, "%.1f degC\nWELCOME", bodyTemp_fr_omv);                 

            // Send increase flag to aws iot
            testReported_Increase = true;
            ESP_LOGI(TAG, "Checker triggered to increase");
        }
        else {  // Set red and exit
            Core2ForAWS_Sk6812_SetBrightness(40);   
            Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_LEFT, 0xbe0000);
            Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_RIGHT, 0xbe0000);
            Core2ForAWS_Sk6812_Show();

            sprintf(temp_str, "%.1f degC\nSORRY..", bodyTemp_fr_omv);
        }
        ui_textlabel_add(temp_str);
        vTaskDelay(pdMS_TO_TICKS(3000));
        Core2ForAWS_Sk6812_Clear();
        Core2ForAWS_Sk6812_Show();
        ui_textlabel_add(NULL);
    }
}

static void uart_rx_task(void *arg){    // receive result from openmv
    int rxBytes;

    char uart_fr_omv[4];
    char status_fr_omv = '0';
    char uart_fr_omv_bodyTemp[3];
    int uart_fr_omv_bodyTemp_int;
    float bodyTemp_fr_omv = 0.0;


    uint8_t *data = heap_caps_malloc(UART_RX_BUF_SIZE, MALLOC_CAP_SPIRAM); // Allocate space for message in external RAM
    while (1) {
        rxBytes = Core2ForAWS_Port_C_UART_Receive(data);

        if (rxBytes >0) {
            for(int i=0; i<4; i++) {
                uart_fr_omv[i] = data[i];              // somehow data seems not flushing buffer after read
            }                                          // and it retain old string in the data
            
            status_fr_omv = uart_fr_omv[0];
            uart_fr_omv_bodyTemp[0] = uart_fr_omv[1];   // somehow how to use this silly method to extract correct value
            uart_fr_omv_bodyTemp[1] = uart_fr_omv[2];
            uart_fr_omv_bodyTemp[2] = uart_fr_omv[3];
            ESP_LOGI(TAG, "uart_fr_omv_bodyTemp\t: %s\n", uart_fr_omv_bodyTemp);

            uart_fr_omv_bodyTemp_int = atoi(uart_fr_omv_bodyTemp);
            ESP_LOGI(TAG, "uart_fr_omv_bodyTemp_int\t: %d\n", uart_fr_omv_bodyTemp_int);

            bodyTemp_fr_omv = (float)uart_fr_omv_bodyTemp_int/10;
            ESP_LOGI(TAG, "Final Body Temperature\t: %.1f degC\n", bodyTemp_fr_omv);
            uart_rx_logic(status_fr_omv, bodyTemp_fr_omv);
            uart_fr_omv[0] = 0;
            uart_fr_omv_bodyTemp[0] = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Read more frequently than transmit to ensure the messages are not erased from buffer.
    }
    free(data); // Free memory from external RAM
}

// *** blink task **************************************************************************
void sk6812_blink_task(void* pvParameters){
    Core2ForAWS_Sk6812_Clear();
    Core2ForAWS_Sk6812_Show();
    Core2ForAWS_Sk6812_SetBrightness(20);
    while (1) {
        
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_LEFT, 0x007acc);
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_RIGHT, 0x007acc);
        Core2ForAWS_Sk6812_Show();
        vTaskDelay(pdMS_TO_TICKS(500));
        Core2ForAWS_Sk6812_Clear();
        Core2ForAWS_Sk6812_Show();
        vTaskDelay(pdMS_TO_TICKS(500));      
    }
    vTaskDelete(NULL); // Should never get to here...
}

// *** iot callback handler ***********************************************************************
void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}

void disconnect_callback_handler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    ui_textarea_add("Disconnected from AWS IoT Core...", NULL, 0);

    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

static bool shadowUpdateInProgress;

void ShadowUpdateStatusCallback(const char *pThingName, ShadowActions_t action, Shadow_Ack_Status_t status,
                                const char *pReceivedJsonDocument, void *pContextData) {
    IOT_UNUSED(pThingName);
    IOT_UNUSED(action);
    IOT_UNUSED(pReceivedJsonDocument);
    IOT_UNUSED(pContextData);

    shadowUpdateInProgress = false;

    if(SHADOW_ACK_TIMEOUT == status) {
        ESP_LOGE(TAG, "Update timed out");
    } else if(SHADOW_ACK_REJECTED == status) {
        ESP_LOGE(TAG, "Update rejected");
    } else if(SHADOW_ACK_ACCEPTED == status) {
        ESP_LOGI(TAG, "Update accepted");
    }
}

// *** desired state callback handler ******************************************************************
void test_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - testDesired state changed to %d", *(bool *) (pContext->pData));
        if(testDesired == false){
            sprintf(temp_str, "We are full now. Sorry . . .");     
            ui_textlabel_add(temp_str);
        } else {
            ui_textlabel_add(NULL);
        }
    }
}

void awsThres_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - thres_fr_aws number changed to %d", *(uint16_t *) (pContext->pData));
        sprintf(temp_str_awsThres, "Threshold: %d", *(uint16_t *) (pContext->pData));
        ui_awsThres_lab(temp_str_awsThres);
    }
}

void awsCount_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - count_fr_aws number changed to %d", *(uint16_t *) (pContext->pData));
        sprintf(temp_str_awsCount, "Counter: %d", *(uint16_t *) (pContext->pData));
        ui_awsCount_lab(temp_str_awsCount);
    }
}

// *** aws iot task ***********************************************************************
void aws_iot_task(void *param) {
    IoT_Error_t rc = FAILURE;

    char JsonDocumentBuffer[MAX_LENGTH_OF_UPDATE_JSON_BUFFER];
    size_t sizeOfJsonDocumentBuffer = sizeof(JsonDocumentBuffer) / sizeof(JsonDocumentBuffer[0]);
    
    // *** json variable ***********************************************************************
    jsonStruct_t testHandler_Increase;
    testHandler_Increase.cb = NULL;
    testHandler_Increase.pKey = "testReported_Increase";
    testHandler_Increase.pData = &testReported_Increase;
    testHandler_Increase.type = SHADOW_JSON_BOOL;
    testHandler_Increase.dataLength = sizeof(bool);

    jsonStruct_t testHandler_Decrease;
    testHandler_Decrease.cb = NULL;
    testHandler_Decrease.pKey = "testReported_Decrease";
    testHandler_Decrease.pData = &testReported_Decrease;
    testHandler_Decrease.type = SHADOW_JSON_BOOL;
    testHandler_Decrease.dataLength = sizeof(bool);

    jsonStruct_t testActuator;
    testActuator.cb = test_Callback;
    testActuator.pKey = "testDesired";
    testActuator.pData = &testDesired;
    testActuator.type = SHADOW_JSON_BOOL;
    testActuator.dataLength = sizeof(bool);

    jsonStruct_t awsThres;
    awsThres.cb = awsThres_Callback;
    awsThres.pKey = "thres_fr_aws";
    awsThres.pData = &thres_fr_aws;
    awsThres.type = SHADOW_JSON_UINT16;
    awsThres.dataLength = sizeof(uint16_t);

    jsonStruct_t awsCount;
    awsCount.cb = awsCount_Callback;
    awsCount.pKey = "count_fr_aws";
    awsCount.pData = &count_fr_aws;
    awsCount.type = SHADOW_JSON_UINT16;
    awsCount.dataLength = sizeof(uint16_t);

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    // initialize the mqtt client
    AWS_IoT_Client iotCoreClient;

    ShadowInitParameters_t sp = ShadowInitParametersDefault;
    sp.pHost = HostAddress;
    sp.port = port;
    sp.enableAutoReconnect = false;
    sp.disconnectHandler = disconnect_callback_handler;

    sp.pRootCA = (const char *)aws_root_ca_pem_start;
    sp.pClientCRT = "#";
    sp.pClientKey = "#0";
    
    #define CLIENT_ID_LEN (ATCA_SERIAL_NUM_SIZE * 2)
    char *client_id = malloc(CLIENT_ID_LEN + 1);
    ATCA_STATUS ret = Atecc608_GetSerialString(client_id);
    if (ret != ATCA_SUCCESS){
        ESP_LOGE(TAG, "Failed to get device serial from secure element. Error: %i", ret);
        abort();
    }

    ui_textarea_add("\nDevice client Id:\n>> %s <<\n", client_id, CLIENT_ID_LEN);

    /* Wait for WiFI to show as connected */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    ESP_LOGI(TAG, "Shadow Init");

    rc = aws_iot_shadow_init(&iotCoreClient, &sp);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_shadow_init returned error %d, aborting...", rc);
        abort();
    }

    ShadowConnectParameters_t scp = ShadowConnectParametersDefault;
    scp.pMyThingName = client_id;
    scp.pMqttClientId = client_id;
    scp.mqttClientIdLen = CLIENT_ID_LEN;

    ESP_LOGI(TAG, "Shadow Connect");
    rc = aws_iot_shadow_connect(&iotCoreClient, &scp);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_shadow_connect returned error %d, aborting...", rc);
        abort();
    }
    ui_wifi_label_update(true); // Switch wifi label to green color
    ui_textarea_add("\nConnected to AWS IoT Core and pub/sub to the device shadow state\n", NULL, 0);

    rc = aws_iot_shadow_set_autoreconnect_status(&iotCoreClient, true);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d, aborting...", rc);
        abort();
    }

    // *** register delta callback  ***************************************************************
    rc = aws_iot_shadow_register_delta(&iotCoreClient, &testActuator);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Shadow Register Delta Error");
    }

    rc = aws_iot_shadow_register_delta(&iotCoreClient, &awsThres);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Shadow Register Delta Error");
    }

    rc = aws_iot_shadow_register_delta(&iotCoreClient, &awsCount);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Shadow Register Delta Error");
    }

    // loop and publish changes
    while(NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc) {
        rc = aws_iot_shadow_yield(&iotCoreClient, 500);
        if(NETWORK_ATTEMPTING_RECONNECT == rc || shadowUpdateInProgress) {
            rc = aws_iot_shadow_yield(&iotCoreClient, 1000);
            // If the client is attempting to reconnect, or already waiting on a shadow update,
            // we will skip the rest of the loop.
            continue;
        }
        // *** Test ***********************************************************************
        ESP_LOGI(TAG, "Just a trigger to see if connection is alive.....................");

        if (startUp == true || testReported_Increase == true || testReported_Decrease == true){

            ESP_LOGI(TAG, "*****************************************************************************************");
            // *** json variable ***********************************************************************
            ESP_LOGI(TAG, "On Device: testReported_Increase %s", testReported_Increase ? "true" : "false");
            ESP_LOGI(TAG, "On Device: testReported_Decrease %s", testReported_Decrease ? "true" : "false");
            ESP_LOGI(TAG, "On Device: testDesired \t\t%s", testDesired ? "true" : "false");
            ESP_LOGI(TAG, "On Device: threshold_fr_aws\t%d", thres_fr_aws);
            ESP_LOGI(TAG, "On Device: count_fr_aws    \t%d", count_fr_aws);

            rc = aws_iot_shadow_init_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
            if(SUCCESS == rc) {
               rc = aws_iot_shadow_add_reported(JsonDocumentBuffer, sizeOfJsonDocumentBuffer, 3,
                                                &testHandler_Increase, &testHandler_Decrease, &testActuator);
                if(SUCCESS == rc) {
                    rc = aws_iot_finalize_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
                    if(SUCCESS == rc) {
                        ESP_LOGI(TAG, "Update Shadow: %s", JsonDocumentBuffer);
                        rc = aws_iot_shadow_update(&iotCoreClient, client_id, JsonDocumentBuffer,
                                                ShadowUpdateStatusCallback, NULL, 6, true);
                        shadowUpdateInProgress = true;
                    }
                }
            }

            ESP_LOGI(TAG, "*****************************************************************************************");
            ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
            
            
            testReported_Increase = false;
            testReported_Decrease = false;
            if (startUp == true) {
                vTaskDelay(pdMS_TO_TICKS(3000));
                startUp = false;
                sprintf(temp_str, "INITIALIZED.");     
                ui_textlabel_add(temp_str);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "An error occurred in the loop %d", rc);
    }

    ESP_LOGI(TAG, "Disconnecting");
    rc = aws_iot_shadow_disconnect(&iotCoreClient);

    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Disconnect error %d", rc);
    }

    vTaskDelete(NULL);
}

// *** touch task for manual trigger test **********************************************************
static void touch_task(void *pvParameters){
    for(;;){
        if(Button_WasReleased(button_left)){
            testReported_Increase = true;
            testReported_Decrease = false;
            ESP_LOGI(TAG, "Button left pressed to increase");
            sprintf(temp_str, "Simulate to INCREASE");     
            ui_textlabel_add(temp_str);
            vTaskDelay(pdMS_TO_TICKS(3500));
            ui_textlabel_add(NULL);
            if(testDesired == false){
                sprintf(temp_str, "We are full now. Sorry . . .");     
                ui_textlabel_add(temp_str);
            } else {
                ui_textlabel_add(NULL);
            }
        }
        if(Button_WasReleased(button_right)){
            testReported_Decrease = true;
            testReported_Increase = false;
            ESP_LOGI(TAG, "Button right pressed to decrease");
            sprintf(temp_str, "Simulate to DECREASE");     
            ui_textlabel_add(temp_str);
            vTaskDelay(pdMS_TO_TICKS(3500));
            ui_textlabel_add(NULL);
            if(testDesired == false){
                sprintf(temp_str, "We are full now. Sorry . . .");     
                ui_textlabel_add(temp_str);
            } else {
                ui_textlabel_add(NULL);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// *** PIR sensor at portB ******************************************************************
void readInputTask(){ 
    bool pir_curr = false;
    bool pir_prev = false;
    int pir_count = 0;
    for(;;){
        pir_curr = Core2ForAWS_Port_Read(GPIO_NUM_26);
        
        if (pir_curr != pir_prev) {
            if (pir_curr == true){
                pir_count++;        // counter to avoid accidental trigger
                if (pir_count > 10){
                    // Send decrease flag to aws iot
                    testReported_Decrease = true;
                    ESP_LOGI(TAG, "PIR sensor triggered!"); 
                    pir_count = 0;
                    pir_prev = pir_curr;
                }
            } else {
                pir_count = 0;
                pir_prev = pir_curr;
            }
        } else{
            pir_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{   
    ESP_LOGI(TAG, "\n***************************************************\nFoolproof Temperature Checker & Crowd Control\n***************************************************");
    Core2ForAWS_Init();
    Core2ForAWS_Display_SetBrightness(50);

    ui_init();
    initialise_wifi();
    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 4096*2, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(touch_task, "touch_Task", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 1);

    esp_err_t err = Core2ForAWS_Port_PinMode(PORT_C_UART_TX_PIN, UART);
    if (err == ESP_OK){
        Core2ForAWS_Port_C_UART_Begin(115200);
        xTaskCreate(uart_rx_task, "uart_rx", 1024*3, NULL, configMAX_PRIORITIES, NULL);
        xTaskCreate(uart_tx_task, "uart_tx", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    }

    esp_err_t err_GPIO26 = Core2ForAWS_Port_PinMode(GPIO_NUM_26, INPUT);
    if(err_GPIO26 == ESP_OK){
        xTaskCreatePinnedToCore(readInputTask, "read_pin", 1024*4, NULL, 1, NULL, 1);
    }

    xTaskCreatePinnedToCore(sk6812_blink_task, "sk6812BlinkTask", configMINIMAL_STACK_SIZE * 3, NULL, 1, &led_bar_blink_handle, 1);
    vTaskSuspend(led_bar_blink_handle);
}