// I2C BME280 sensor - Temperature read

// Tutorial: 			https://esp32tutorials.com/esp32-mqtt-publish-bme280-node-red-esp-idf/#more-2125
// Original example: 		https://github.com/ESP32Tutorials/esp32-esp-idf-mqtt-bme280/blob/main/main/main.c
// BME280 files:		https://github.com/ESP32Tutorials/esp32-esp-idf-mqtt-bme280/tree/main/components/bme280

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#include "bme280.h"
#include "sdkconfig.h"
#include "HD44780.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#define TAG_BME280 "BME280"

#define BME_SDA_PIN GPIO_NUM_21
#define BME_SCL_PIN GPIO_NUM_22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

// #define LCD_ADDR 0x27
// #define LCD_SDA_PIN  23
// #define LCD_SCL_PIN  25
// #define LCD_COLS 16
// #define LCD_ROWS 2
#define GPIO_PIN 19

// Hàm cấu hình chân GPIO 19 thành đầu vào digital
// void configureGPIO() {
//     gpio_config_t io_config;
//     io_config.intr_type = GPIO_INTR_DISABLE;     // Tắt chức năng ngắt
//     io_config.mode = GPIO_MODE_INPUT;            // Cấu hình chế độ đầu vào
//     io_config.pin_bit_mask = (1ULL << GPIO_PIN); // Đặt chân GPIO
//     io_config.pull_down_en = GPIO_PULLDOWN_DISABLE; // Không kích kéo xuống
//     io_config.pull_up_en = GPIO_PULLUP_ENABLE;   // Kích kéo lên
//     gpio_config(&io_config);                     // Áp dụng cấu hình
// }
static const char *TAG = "MoistureSensor";

// void soilData(){
// 	adc1_config_width(ADC_WIDTH_BIT_10);       // Cấu hình độ phân giải ADC 10 bit (0-1023)
//     adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);  // Cấu hình kênh ADC và mức giảm áp (chân GPIO 32)
//     esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
//     esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_10, 1100, adc_chars);
//     if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
//         ESP_LOGI(TAG, "ADC calibration value efuse tp");
//     } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
//         ESP_LOGI(TAG, "ADC calibration value efuse vref");
//     } else {
//         ESP_LOGI(TAG, "ADC calibration value default");
//     }

//     while (1) {
//         int value = adc1_get_raw(ADC1_CHANNEL_4);  // Đọc giá trị từ cảm biến (chân GPIO 32)
//         int percent = 100 - (value * 100 / 1023);  // Chuyển đổi giá trị thành phần trăm

//         printf("Do am: %d\n", percent);
//         vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 giây
//     }
//  }
// Initialize I2C communication parameters: khởi tạo giao tiếp i2c
void i2c_master_init()
{	//tạo biến cấu trúc i2c_config: chứa thông tin về cấu hình giao tiếp i2c
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER, // thiết lập chế độ giao tiếp là chế độ master
		.sda_io_num = BME_SDA_PIN, // thiết lập chân GPIO là chân dữ liệu SDA
		.scl_io_num = BME_SCL_PIN, 
		.sda_pullup_en = GPIO_PULLUP_ENABLE, // kích hoạt điện trở kéo lên pullup để đảm bảo tín hiệu ổn định
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000}; // thiết lập tốc độ truyền dữ liệu là 1Mhz
	i2c_param_config(I2C_NUM_0, &i2c_config); // cấu hình tham số i2c cho controller
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0); //cài đặt driver i2c cho i2c controller
}

// BME280 I2C write function
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) //hàm trả về giá trị kiểu s8
// địa chỉ thiết bị, địa chỉ thanh ghi, con trỏ tới dữ liệu, số lượng byte cần gửi:cnt
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Tạo hàm handle 'cmd' xây dựng chuỗi lệnh gửi đến bme280
	i2c_master_start(cmd); // thêm lệnh khởi động truyền dữ liệu, bắt đầu với TH start
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	//thêm lệnh gửi địa chỉ thiết bị bằng cách dịch trái dev_addr 1 để thêm bit R/W
	i2c_master_write_byte(cmd, reg_addr, true);// thêm lệnh gửi địa chỉ thanh ghi 
	i2c_master_write(cmd, reg_data, cnt, true);// thêm lệnh gửi dữ liệu con trỏ
	i2c_master_stop(cmd);// thêm lệnh kết thúc truyền

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS); 
	//thực thi chuỗi lệnh i2c controller, thời gian chờ là 10ms
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C read function
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
	//kiểm tra số lượng byte cần đọc, nếu >1 thì đọc các byte dữ liệu trừ byte cuối
	if (cnt > 1)
	{
		i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C delay function: tạo độ trễ theo thời gian được chỉ định, đv là mili giây
void BME280_delay_msek(u32 msek) //nhận đối số msek
{
	vTaskDelay(msek / portTICK_PERIOD_MS);
}

// BME280 I2C task: task chạy trên freeRTOS, đọc dữ liệu từ bme280 và in ra màn hình
void Publisher_Task(void *params)
{
	// BME280 I2C communication structure
	struct bme280_t bme280 = {
		.bus_write = BME280_I2C_bus_write, //con trở tới hàm ghi
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS1,	//địa chỉ i2c của bme280
		.delay_msec = BME280_delay_msek};	// con trỏ tới hàm tạo độ trễ

	s32 com_rslt; // giá trị trả về
	//khai báo các biến cục bộ chưa được xử lý
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;

	// Initialize BME280 sensor and set internal parameters: thiết lập cấu hình ban đầu
	com_rslt = bme280_init(&bme280);
	printf("com_rslt %d\n", com_rslt);

	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);
	//thiết lập thời gian chờ giữa các lần đọc dữ liệu từ cảm biến
	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	// thiết lập bộ lọc dữ liệu cho cảm biến
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);
	// thiết lập chế độ hoạt động cho cảm biến: normal
	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);	

	//đọc cảm biến độ ẩm đất
	adc1_config_width(ADC_WIDTH_BIT_10);       // Cấu hình độ phân giải ADC 10 bit (0-1023)
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);  // Cấu hình kênh ADC và mức giảm áp (chân GPIO 32)
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_10, 1100, adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "ADC calibration value efuse tp");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "ADC calibration value efuse vref");
    } else {
        ESP_LOGI(TAG, "ADC calibration value default");
    }

	if (com_rslt == SUCCESS)
	{
		while (true)
		{
			vTaskDelay(10000 / portTICK_PERIOD_MS);
			// int value = adc1_get_raw(ADC1_CHANNEL_4);  // Đọc giá trị từ cảm biến (chân GPIO 32)
			// int percent = 100 - (value * 100 / 1023);  // Chuyển đổi giá trị thành phần trăm
			// printf("soil: %d, ", percent);
			// int value = gpio_get_level(GPIO_PIN);
			// printf("soil %d ",value);
			int value = adc1_get_raw(ADC1_CHANNEL_4);  // Đọc giá trị từ cảm biến (chân GPIO 32)
			int percent = 100 - (value * 100 / 1023);  // Chuyển đổi giá trị thành phần trăm
			printf("Soil: %d%%\n", percent);
			// Read BME280 data
			com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
				&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);
			//hàm xử lý dữ liệu uncomp/raw
			double temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
			char temperature[12];
			sprintf(temperature, "%.2f degC", temp);

			double press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
			char pressure[10];
			sprintf(pressure, "%.2f hPa", press);

			double hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
			char humidity[10];
			sprintf(humidity, "%.2f %%", hum);
			
			// Print BME data
			if (com_rslt == SUCCESS)
			{	

				printf("Temperature %s, Pressure %s, Humidity %s\n",temperature, pressure, humidity);
				
			}
			else
			{
				ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
			}
		}
	}
	else
	{
		ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
	}
}
void app_main(void) // khởi động esp
{
	// Initialize memory: khởi tạo bộ nhớ non volatile storage/ bộ nhớ điện tĩnh
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase()); //hàm xóa toàn bộ dữ liệu trong nvs
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret); // kiểm tra kết quả khởi tạo nvs

	// Initialize I2C parameters: khởi tạo thông số
	i2c_master_init();
	// LCD_init(LCD_ADDR, LCD_SDA_PIN, LCD_SCL_PIN, LCD_COLS, LCD_ROWS);
	//gọi hàm độ ẩm đất
	// Đọc giá trị từ chân GPIO 19
	// Read the data from BME280 sensor
	//khởi động task freeRTOS, với kích thước 5kb, độ ưu tiên (configMAX_PRIORITIES-1) là 5
	xTaskCreate(&Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL); // NULL được truyền cho đối số params

}
