#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    3300        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define BATTERY_VOLTAGE_FACTOR 1.27

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t battery_cell1 = ADC_CHANNEL_7;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t battery_cell2 = ADC_CHANNEL_4;
static const adc_channel_t battery_cell3 = ADC_CHANNEL_5;

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;


float battery_percentages[] = {100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5,0};
float battery_levels[] = {4.2, 4.15, 4.11, 4.08, 4.02, 3.98, 3.95, 3.91, 3.87, 3.85, 3.84, 3.82 ,3.8, 3.79, 3.77, 3.75, 3.73, 3.71, 3.69, 3.61, 3.27};
// 3.71 - yellow - 15 percent
// 3.69 - red - 10 percent


void adc_init(){
    adc1_config_width(width);
    adc1_config_channel_atten(battery_cell1, atten);
    adc1_config_channel_atten(battery_cell2, atten);
    adc1_config_channel_atten(battery_cell3, atten);
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

void update_battery_voltage(){
    uint32_t adc_reading1 = 0;
    uint32_t adc_reading2 = 0;
    uint32_t adc_reading3 = 0;

    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading1 += adc1_get_raw((adc1_channel_t)battery_cell1);
            adc_reading2 += adc1_get_raw((adc1_channel_t)battery_cell2);
            adc_reading3 += adc1_get_raw((adc1_channel_t)battery_cell3);
    }
    adc_reading1 /= NO_OF_SAMPLES;
    adc_reading2 /= NO_OF_SAMPLES;
    adc_reading3 /= NO_OF_SAMPLES;

    //Convert adc_reading to voltage in mV
    uint32_t voltage1 = esp_adc_cal_raw_to_voltage(adc_reading1, adc_chars);
    uint32_t voltage2 = esp_adc_cal_raw_to_voltage(adc_reading2, adc_chars);
    uint32_t voltage3 = esp_adc_cal_raw_to_voltage(adc_reading3, adc_chars);

    printf("Raw: %d\tVoltage1: %dmV\n", adc_reading1, voltage1);
    printf("Raw: %d\tVoltage2: %dmV\n", adc_reading2, voltage2);
    printf("Raw: %d\tVoltage3: %dmV\n", adc_reading3, voltage3);
}

void app_main(void)
{
    //Configure ADC
    adc_init();

    //Continuously sample ADC1
    while (1) {
        update_battery_voltage();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}