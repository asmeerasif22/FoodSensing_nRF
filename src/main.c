#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nsms.h>

#define SLEEP_TIME_MS       3000
#define ADC_NUM_CHANNELS    5
#define GPIO1_NODE          DT_NODELABEL(gpio1)

//Additional code for BLE
#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL),
};
static struct bt_conn *current_conn = NULL;
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}
    current_conn = bt_conn_ref(conn);

	printk("Connected\n");

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
    if(current_conn){
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
	dk_set_led_off(CON_STATUS_LED);
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
};


static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
//BLE Basic Code end

//BLE Custom Code Start
#define BUFF_SIZE 64
BT_NSMS_DEF(nsms_amount, "Amount", false, "Unknown", BUFF_SIZE);

BT_CONN_CB_DEFINE(conn_callbacks1) = {
    .connected = connected,
    .disconnected = disconnected,
};

static bool send_amount(float value, const char *name){
    char buf[BUFF_SIZE];
    int len = sprintf(buf,"%s", name);
    len += sprintf(buf + len, " %.6f", (double)value); 
    bt_nsms_set_status(&nsms_amount, buf);
    return true;
}
//BLE Custom Code End

//BLE Receive Code
#define BT_UUID_TRIGGER_SERVICE_VAL  BT_UUID_128_ENCODE(0xca28ebfb, 0xc4ec, 0x40fd, 0x83db, 0xd260036fe623)
#define BT_UUID_TRIGGER_SERVICE      BT_UUID_DECLARE_128(BT_UUID_TRIGGER_SERVICE_VAL)

#define BT_UUID_TRIGGER_CHAR_VAL  BT_UUID_128_ENCODE(0xca28ebfc, 0xc4ec, 0x40fd, 0x83db, 0xd260036fe623) // Slightly changed last digit
#define BT_UUID_TRIGGER_CHAR      BT_UUID_DECLARE_128(BT_UUID_TRIGGER_CHAR_VAL)

static bool trigger_measurement = false;

static ssize_t write_trigger(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len,
                             uint16_t offset, uint8_t flags)
{
    if (len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    trigger_measurement = *((uint8_t *)buf);
    printk("Trigger received: %s\n", trigger_measurement ? "ON" : "OFF");

    return len;
}

// Define the BLE GATT service
BT_GATT_SERVICE_DEFINE(trigger_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_TRIGGER_SERVICE),

    BT_GATT_CHARACTERISTIC(BT_UUID_TRIGGER_CHAR,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, write_trigger, NULL),

);

//BLE Receive Code End

//DEFINING GPIO PINS
static const uint8_t GPIO_PT[5] = {1, 2, 3, 4, 5};
static const uint8_t GPIO_LED[5] = {6, 7, 8, 10,11};



//DEFINING ADC CHANNELS
static const struct adc_dt_spec adc_channels[ADC_NUM_CHANNELS] = {
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 4)
};
static int16_t adc_buffers[ADC_NUM_CHANNELS];

//Helper function to get the height of the food in the container using the polynomial model
double get_height(const int sensor_value, const double coefficients[], int sensor_id) {
    double a = coefficients[0];
    double b = coefficients[1];
    double c = coefficients[2] - sensor_value; // Adjust c to include sensor value
    double discriminant = (b * b) - (4.0 * a * c);

    // Check if the discriminant is negative (no real roots)
    if (discriminant < 0) {
        printf("**No real roots for sensor %d value: %d\n", sensor_id, sensor_value);
        return -1; // Invalid height
    }

    // Calculate roots
    double root1 = (-b + sqrt(discriminant)) / (2.0 * a);
    double root2 = (-b - sqrt(discriminant)) / (2.0 * a);

    // Return the positive root (if any)
    if (root1 > 0 && root2 > 0) {
        return fmin(root1, root2); // Choose the smaller positive root
    } else if (root1 > 0) {
        return root1;
    } else if (root2 > 0) {
        return root2;
    } else {
        printf("**No positive roots for sensor %d value: %d\n", sensor_id, sensor_value);
        return -1; // No valid positive root
    }
}

//FUNCTION TO GET AMMOUNT OF FOOD IN THE CONTAINER
double get_amount(const int arr[]){
    
    const int container_height = 16.0;
    const double container_radius = 6.25;
    const double useable_height = 15.5;
    const double food_density = 0.42058;
    
    const int s1 = arr[0];
    const int s2 = arr[1];
    const int s3 = arr[2];
    const int s4 = arr[3];
    const int s5 = arr[4];

    //Polynomial models for each sensor
    const double p1[3] = {2.20, -87.12, 1014.69};
    const double p2[3] = {6.83, -202.2, 1730.19};
    const double p3[3] = {0.59, -43.70, 674.78};
    const double p4[3] = {5.68, -179.04, 1602.30};
    const double p5[3] = {10.18, -303.72, 2611.28};

    //Getting height from polynomial models. 
    double h1 = get_height(s1,p1,1);
    double h2 = get_height(s2,p2,2);
    double h3 = get_height(s3,p3,3);
    double h4 = get_height(s4,p4,4);
    double h5 = get_height(s5,p5,5);

    double h_avg = (h1+h2+h3+h4+h5)/5;
    double h_food = container_height - h_avg;
    printf("Average height of food in the container: %lf cm\n", h_food);

    double full_percent = (h_food/useable_height)*100;
    printf("Percentage full: %lf %%\n", full_percent);

    double food_amount = h_food*(22.0/7.0)*(container_radius*container_radius)*food_density;
    printf("Estimated food amount: %lf g\n", food_amount);

    printf("\n\nHeight from top (for verification): %lf cm\n", h_avg);
    return full_percent;

}

//MAIN FUNCTION
int main(void)
{   //BLE Code
    int blink_status = 0;
	int err;

	printk("Starting Bluetooth Peripheral LBS sample\n");

	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_BT_LBS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}
    //BLE Code end

    int ret;
    double amount;
    int all_reads[5];
    // Check and setup each channel
    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
        if (!adc_is_ready_dt(&adc_channels[i])) {
            printk("ADC controller device %s for channel %d not ready\n",
                   adc_channels[i].dev->name, adc_channels[i].channel_id);
            return -1;
        }

        ret = adc_channel_setup_dt(&adc_channels[i]);
        if (ret < 0) {
            printk("Failed to setup ADC channel %d (%d)\n", i, ret);
            return -1;
        }
    }

    // ADC sequence configuration (common for all channels)
    struct adc_sequence sequence = {
        .resolution = adc_channels[0].resolution,
        .buffer_size = sizeof(int16_t), // Single sample per read
    };

	const struct device *gpio_dev = device_get_binding("gpio@50000300");
    printk("\nGPIO Device Binding completed\n");
    for(int x = 0; x < 5; x++){
        gpio_pin_configure(gpio_dev, GPIO_PT[x], GPIO_OUTPUT_LOW);

    }
    printk("GPIO PT PINS CONFIGURED\n");

        for(int x = 0; x < 5; x++){
        gpio_pin_configure(gpio_dev, GPIO_LED[x], GPIO_OUTPUT_LOW);

    }
    printk("GPIO LED PINS CONFIGURED\n");

    while (1) {
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
        if (trigger_measurement) {  // Only read if triggered by phone
            printk("Trigger activated, taking measurement...\n");
            trigger_measurement = false;  // Reset trigger
            printk("\n_________________________\n\nMEASUREMENT INITIATED\n");
            printk("\nPlease Wait\n_________________________\n\n");

            // Loop to activate each channel and read the ADC value.
            for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
                // Setting one PT pin high and the rest low
                for(int j = 0; j < 5; j++){
                    if(j == i){
                        gpio_pin_set(gpio_dev, GPIO_PT[j], 1);
                    }
                    else{
                        gpio_pin_set(gpio_dev, GPIO_PT[j], 0);
                    }
                }
                //printk("PT %d turned on\n", i+1);

                // Setting one LED pin high and the rest low
                for(int j = 0; j < 5; j++){
                    if(j == i){
                        gpio_pin_set(gpio_dev, GPIO_LED[j], 1);
                    }
                    else{
                        gpio_pin_set(gpio_dev, GPIO_LED[j], 0);
                    }
                }
            //printk("LED %d turned on\n", i+1);

                sequence.buffer = &adc_buffers[i];
                sequence.channels = BIT(adc_channels[i].channel_id);
                k_sleep(K_MSEC(SLEEP_TIME_MS));

                // Read the ADC value
                ret = adc_read(adc_channels[i].dev, &sequence);
                if (ret < 0) {
                    printk("Failed to read ADC channel %d (%d)\n", i, ret);
                    continue;
                }

                // Convert raw ADC value to millivolts
                int val_mv = (int)adc_buffers[i];
                ret = adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
                if (ret < 0) {
                    printk("Channel %d: Raw: %d (mV conversion unavailable)\n", i, adc_buffers[i]);
                } else {
                    printk("Channel %d: Raw: %d, Voltage: %d mV\n\n\n", i+1, adc_buffers[i], val_mv);
                    all_reads[i] = val_mv;
                    
                }
                //k_sleep(K_MSEC(SLEEP_TIME_MS));
            }

            //Edge case for empty container
            if(all_reads[0] > 900 || all_reads[1] > 1000 || all_reads[2] > 700 || all_reads[3] > 900 || all_reads[4] > 2000){
                printk("\n____________________\n\nContainer is empty\n");
                amount = 0.0;
                if(current_conn != NULL){
                    send_amount(amount, "Amount");
                }
                for(int j = 0; j < 5; j++){
                    gpio_pin_set(gpio_dev, GPIO_LED[j], 0);
                }
                //return 0;
            }
            else{

            //Computing the amount of food in the container
                amount = get_amount(all_reads);
                if(current_conn != NULL){
                    send_amount(amount, "Amount");
                }
                for(int j = 0; j < 5; j++){
                    gpio_pin_set(gpio_dev, GPIO_LED[j], 0);
                }

            //return 0;
            }
        }

    }

    return 0;
}