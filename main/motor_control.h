static inline uint32_t bldc_get_hall_sensor_value(bool ccw);
void update_bldc_speed();
void update_hardware();
static void pwm_init(void);
void periodic_speed_update_func_init();
static void IRAM_ATTR gpio_isr_handler_func(void* arg);
void hall_sensor_init();
void enable_pin_init();
//put the motor control code in the core1 task
void core1_tasks(void *params);
