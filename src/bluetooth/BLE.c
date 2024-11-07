/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
/** @file
 *  @brief LED Button Service (LBS) sample
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <hal/nrf_saadc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/BLE.h>
#include <zephyr/logging/log.h>
volatile int cN =0;

// LOG_MODULE_REGISTER(bt_lbs, CONFIG_BT_LBS_LOG_LEVEL);
LOG_MODULE_REGISTER(bt_lbs);

static bool                   notify_enabled;
static bool                   button_state;
static struct bt_lbs_cb       lbs_cb;

static void lbslc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	
	cN = 1;
	printk(" enabling notification %d\n",cN);
}

static ssize_t write_led(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle,
		(void *)conn);

	if (len != 1U) {
		LOG_DBG("Write led: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_DBG("Write led: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (lbs_cb.led_cb) {
		uint8_t val = *((uint8_t *)buf);

		if (val == 0x00 || val == 0x01) {
			lbs_cb.led_cb(val ? true : false);
		} else {
			LOG_DBG("Write led: Incorrect value");
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
	}

	return len;
}

#ifdef CONFIG_BT_LBS_POLL_BUTTON
static ssize_t read_button(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	const char *value = attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle,
		(void *)conn);

	if (lbs_cb.button_cb) {
		button_state = lbs_cb.button_cb();
		return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
					 sizeof(*value));
	}

	return 0;
}
#endif

/* LED Button Service Declaration */
BT_GATT_SERVICE_DEFINE(lbs_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS),
#ifdef CONFIG_BT_LBS_POLL_BUTTON
	BT_GATT_CHARACTERISTIC(BT_UUID_ECG,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_button, NULL,
			       &button_state),
#else
	BT_GATT_CHARACTERISTIC(BT_UUID_ECG,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
#endif
	BT_GATT_CCC(lbslc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),	
			BT_GATT_CHARACTERISTIC(BT_UUID_HR,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),	
				   	BT_GATT_CCC(lbslc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
			BT_GATT_CHARACTERISTIC(BT_UUID_HRV,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			      BT_GATT_PERM_READ,
			       NULL, NULL, NULL),
			BT_GATT_CCC(lbslc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

int bt_lbs_init(struct bt_lbs_cb *callbacks)
{
	if (callbacks) {
		lbs_cb.led_cb    = callbacks->led_cb;
		lbs_cb.button_cb = callbacks->button_cb;
	}

	return 0;
}
 

int notifyHR(int16_t my_data[], uint8_t n){
     uint8_t readSamples = n * 2;
	 uint8_t arr[readSamples];
    //  printk("ECG notified");
    for(int i = 0; i< n ; i++){
		arr[i * 2] = (my_data[i]) >> 8;
		arr[2 * i + 1] =(my_data[i] );
        // int16_t r = (arr[i * 2]) ;
        // r = r << 8;
        // r = r | arr[i* 2 + 1];
        // printk("%6d - %6d\n",my_data[i], r);           
    }
	 return bt_gatt_notify(NULL, &lbs_svc.attrs[5],
			     arr,
			      readSamples * sizeof(uint8_t));
}

int notifyECG(int16_t my_data[], uint8_t n){
     uint8_t readSamples = n * 2;
	 uint8_t arr[readSamples];
    //  printk("ECG notified");
    for(int i = 0; i< n ; i++){
		arr[i * 2] = (my_data[i]) >> 8;
		arr[2 * i + 1] =(my_data[i] );
        // int16_t r = (arr[i * 2]) ;
        // r = r << 8;
        // r = r | arr[i* 2 + 1];
        // printk("%6d - %6d\n",my_data[i], r);           
    }
	 return bt_gatt_notify(NULL, &lbs_svc.attrs[2],
			     arr,
			      readSamples * sizeof(uint8_t));
}

int notifyHRV(uint8_t my_data[]){
	
	 return bt_gatt_notify(NULL, &lbs_svc.attrs[8],
			     my_data,
			      10 * sizeof(uint8_t));
}
