#include <rmw_microros/rmw_microros.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- USB CDC Handles ---
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
extern USBD_HandleTypeDef hUsbDeviceHS;

// --- Reimplemented USB CDC callbacks ---
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len);

// Line coding: Rate: 115200bps; CharFormat: 1 Stop bit; Parity: None; Data: 8 bits
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

// --- micro-ROS Transports ---
#define USB_RX_BUFFER_SIZE 8192U
#define USB_TX_BUFFER_SIZE 16384U
#define USB_TX_CHUNK_SIZE 512U
#define USB_TX_COMPLETE_TIMEOUT_MS 100U
#define USB_TX_TASK_STACK_WORDS 512U

static uint8_t storage_buffer[USB_RX_BUFFER_SIZE]
    __attribute__((section(".usb_transport"), aligned(32)));
static uint8_t tx_buffer[USB_TX_BUFFER_SIZE]
    __attribute__((section(".usb_transport"), aligned(32)));
static StackType_t tx_task_stack[USB_TX_TASK_STACK_WORDS]
    __attribute__((section(".usb_transport"), aligned(32)));
static StaticTask_t tx_task_tcb
    __attribute__((section(".usb_transport"), aligned(32)));
static StaticSemaphore_t tx_data_semaphore_storage;
static StaticSemaphore_t tx_complete_semaphore_storage;
static SemaphoreHandle_t tx_data_semaphore;
static SemaphoreHandle_t tx_complete_semaphore;
static TaskHandle_t tx_task_handle;
static volatile size_t it_head;
static volatile size_t it_tail;
static volatile size_t tx_head;
static volatile size_t tx_tail;
volatile uint32_t g_usb_rx_overflow_count = 0;
volatile uint32_t g_usb_write_timeout_count = 0;
volatile uint32_t g_usb_tx_overflow_count = 0;
volatile uint32_t g_usb_tx_high_water_mark = 0;
static bool initialized = false;

static size_t ring_used(size_t head, size_t tail, size_t capacity)
{
    return (tail >= head) ? (tail - head) : (capacity - head + tail);
}

static void usb_tx_task(void *argument)
{
    (void)argument;
    for (;;) {
        size_t head = tx_head;
        size_t tail = tx_tail;
        if (head == tail) {
            (void)xSemaphoreTake(tx_data_semaphore, portMAX_DELAY);
            continue;
        }
        if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) {
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        size_t contiguous = (tail > head) ? (tail - head) :
            (USB_TX_BUFFER_SIZE - head);
        if (contiguous > USB_TX_CHUNK_SIZE) {
            contiguous = USB_TX_CHUNK_SIZE;
        }
        (void)xSemaphoreTake(tx_complete_semaphore, 0U);
        if (CDC_Transmit_HS(&tx_buffer[head], (uint16_t)contiguous) !=
            USBD_OK) {
            vTaskDelay(pdMS_TO_TICKS(1U));
            continue;
        }
        if (xSemaphoreTake(tx_complete_semaphore,
                           pdMS_TO_TICKS(USB_TX_COMPLETE_TIMEOUT_MS)) ==
            pdTRUE) {
            __DMB();
            tx_head = (head + contiguous) % USB_TX_BUFFER_SIZE;
        } else {
            g_usb_write_timeout_count++;
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
    }
}

// Transmission completed callback
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    (void) Buf;
    (void) Len;
    (void) epnum;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (tx_complete_semaphore != NULL) {
        xSemaphoreGiveFromISR(tx_complete_semaphore,
                              &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
    return USBD_OK;
}

// USB CDC requests callback
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    switch(cmd)
    {
        case CDC_SET_LINE_CODING:
        memcpy(line_coding, pbuf, sizeof(line_coding));
        break;

        case CDC_GET_LINE_CODING:
        memcpy(pbuf, line_coding, sizeof(line_coding));
        break;

        case CDC_SEND_ENCAPSULATED_COMMAND:
        case CDC_GET_ENCAPSULATED_RESPONSE:
        case CDC_SET_COMM_FEATURE:
        case CDC_GET_COMM_FEATURE:
        case CDC_CLEAR_COMM_FEATURE:
        case CDC_SET_CONTROL_LINE_STATE:
        case CDC_SEND_BREAK:
        default:
            break;
    }

    return USBD_OK;
}

// Data received callback
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
		USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);

		/* ISR producer, micro-ROS task consumer. Keep one byte empty so
		 * head == tail always means empty. Drop a complete USB packet when it
		 * cannot fit; a partial XRCE packet would corrupt stream framing. */
		size_t head = it_head;
		size_t tail = it_tail;
		size_t used = (tail >= head) ? (tail - head)
		                            : (USB_RX_BUFFER_SIZE - head + tail);
		size_t free_bytes = USB_RX_BUFFER_SIZE - used - 1U;
		if ((size_t)*Len <= free_bytes) {
			for (uint32_t i = 0U; i < *Len; i++) {
				storage_buffer[tail] = Buf[i];
				tail = (tail + 1U) % USB_RX_BUFFER_SIZE;
			}
			__DMB();
			it_tail = tail;
		} else {
			g_usb_rx_overflow_count++;
		}

	USBD_CDC_ReceivePacket(&hUsbDeviceHS);

	return (USBD_OK);
}

bool cubemx_transport_open(struct uxrCustomTransport * transport){
	(void)transport;

    if (!initialized)
    {
		memset(storage_buffer, 0, sizeof(storage_buffer));
		memset(tx_buffer, 0, sizeof(tx_buffer));
		it_head = 0U;
		it_tail = 0U;
		tx_head = 0U;
		tx_tail = 0U;
		tx_data_semaphore = xSemaphoreCreateBinaryStatic(
			&tx_data_semaphore_storage);
		tx_complete_semaphore = xSemaphoreCreateBinaryStatic(
			&tx_complete_semaphore_storage);
		if (tx_data_semaphore == NULL || tx_complete_semaphore == NULL) {
			return false;
		}
		tx_task_handle = xTaskCreateStatic(
			usb_tx_task, "usbTx", USB_TX_TASK_STACK_WORDS, NULL,
			(UBaseType_t)osPriorityAboveNormal, tx_task_stack, &tx_task_tcb);
		if (tx_task_handle == NULL) {
			return false;
		}
        // USB is initialized on generated main code: Replace default callbacks here
        USBD_Interface_fops_HS.Control = CDC_Control_HS;
        USBD_Interface_fops_HS.Receive = CDC_Receive_HS;
        USBD_Interface_fops_HS.TransmitCplt = CDC_TransmitCplt_HS;
        initialized = true;
    }

    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
	(void)transport;
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
		(void)transport;
		if (err != NULL) {
			*err = 0U;
		}
		if (buf == NULL || len == 0U || len >= USB_TX_BUFFER_SIZE ||
		    tx_data_semaphore == NULL) {
			if (err != NULL) {
				*err = 1U;
			}
			return 0;
		}

		size_t head = tx_head;
		size_t tail = tx_tail;
		size_t used = ring_used(head, tail, USB_TX_BUFFER_SIZE);
		if (len > (USB_TX_BUFFER_SIZE - used - 1U)) {
			g_usb_tx_overflow_count++;
			if (err != NULL) {
				*err = 1U;
			}
			return 0U;
		}
		size_t first = USB_TX_BUFFER_SIZE - tail;
		if (first > len) {
			first = len;
		}
		memcpy(&tx_buffer[tail], buf, first);
		if (len > first) {
			memcpy(tx_buffer, &buf[first], len - first);
		}
		__DMB();
		tx_tail = (tail + len) % USB_TX_BUFFER_SIZE;
		used += len;
		if (used > g_usb_tx_high_water_mark) {
			g_usb_tx_high_water_mark = (uint32_t)used;
		}
		xSemaphoreGive(tx_data_semaphore);
		return len;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
	(void)transport;
	if (err != NULL) {
		*err = 0U;
	}

    int64_t start = uxr_millis();
    size_t readed = 0;

    do
    {
        if (it_head != it_tail)
        {
            while ((it_head != it_tail) && (readed < len)){
                buf[readed] = storage_buffer[it_head];
                it_head = (it_head + 1) % USB_RX_BUFFER_SIZE;
                readed++;
            }

            break;
        }

       osDelay(1);
    } while ((uxr_millis() -  start) < timeout);

    return readed;
}

uint32_t microros_usb_rx_overflow_count(void)
{
	return g_usb_rx_overflow_count;
}

uint32_t microros_usb_write_timeout_count(void)
{
	return g_usb_write_timeout_count;
}

uint32_t microros_usb_tx_overflow_count(void)
{
	return g_usb_tx_overflow_count;
}

uint32_t microros_usb_tx_high_water_mark(void)
{
	return g_usb_tx_high_water_mark;
}

#endif
