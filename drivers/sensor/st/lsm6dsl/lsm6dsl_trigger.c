/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_lsm6dsl

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include "lsm6dsl.h"
#include "math.h"

LOG_MODULE_DECLARE(LSM6DSL, CONFIG_SENSOR_LOG_LEVEL);

static inline void setup_irq(const struct device *dev, bool enable)
{
	const struct lsm6dsl_config *config = dev->config;

	unsigned int flags = enable
		? GPIO_INT_EDGE_TO_ACTIVE
		: GPIO_INT_DISABLE;

	gpio_pin_interrupt_configure_dt(&config->int_gpio, flags);
}

static inline void handle_irq(const struct device *dev)
{
	struct lsm6dsl_data *drv_data = dev->data;

	setup_irq(dev, false);

	#if defined(CONFIG_LSM6DSL_TRIGGER_OWN_THREAD)
		k_sem_give(&drv_data->gpio_sem);
	#elif defined(CONFIG_LSM6DSL_TRIGGER_GLOBAL_THREAD)
		k_work_submit(&drv_data->work);
	#endif
}

int lsm6dsl_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler){

	const struct lsm6dsl_config *config = dev->config;
	struct lsm6dsl_data *drv_data = dev->data;
	uint8_t register_value;

	LOG_DBG("lsm6dsl_trigger_set type : %d", trig->type);	
	switch (trig->type) {
		case SENSOR_TRIG_THRESHOLD:
			LOG_DBG("trigger type is: SENSOR_TRIG_THRESHOLD");
			/* It uses the device wakeup feature */
			
			//Write 60h to CTRL1_XL turn on the Accelerometer*/
			lsm6dsl_pm_action(dev, PM_DEVICE_ACTION_RESUME);
			
			/* Enable interrutps and apply high pass digital filer, latched */
			//Write 90h to TAP_CFG
			register_value = LSM6DSL_MASK_TAP_CFG_INTERRPUTS_ENABLE | LSM6DSL_MASK_TAP_CFG_SLOPE_FDS;
			LOG_DBG("SET LSM6DSL_REG_TAP_CFG 0x%x ", register_value);
			if (drv_data->hw_tf->write_data(dev,
						LSM6DSL_REG_TAP_CFG,
						&register_value, 1) < 0) {
				LOG_ERR("Could not Set TAP_CFG register");
				return -EIO;			
			}

			/*Write 00h to WAKE_UP_DUR // No duration */
			register_value = 0x00;
			LOG_DBG("SET LSM6DSL_REG_WAKE_UP_DUR 0x%x ", register_value);
			if (drv_data->hw_tf->write_data(dev,
						LSM6DSL_REG_WAKE_UP_DUR,
						&register_value, 1) < 0) {
				LOG_ERR("Could not Set WAKE_UP_DUR register");
				return -EIO;			
			}

			/*Write 02h to WAKE_UP_THS // Set wake-up threshold */
			LOG_WRN("Need to make the threshold configurable");			
			//THREHOLD =  62.5 mg (= 2 * FS_XL / 2^6)
			double threshold = 2.0 * ((double) drv_data->accel_fs) / 64.0;			
			double requested_threshold_ms2 = (double) (drv_data->accel_upper_threshold_ms2);
			double requested_threshold_g = ( (requested_threshold_ms2) / SENSOR_G ) * 1E6;
			double threshold_g = (requested_threshold_g * 64.0 ) / ((double) drv_data->accel_fs);
			register_value = (uint8_t) roundl(threshold_g);
			LOG_DBG("Requested threshold is %i [mms2]  ->  %i [mg].  RegDouble %i [x1E3]  Reg: %u", 
												(int) (drv_data->accel_upper_threshold_ms2 * 1000.0), 
												(int) (requested_threshold_g * 1000.0),
												(int) (threshold_g * 1000),
												register_value);
								
			
			
			if (drv_data->hw_tf->write_data(dev,
						LSM6DSL_REG_WAKE_UP_THS,
						&register_value, 1) < 0) {
				LOG_ERR("Could not Set WAKE_UP_DUR register");
				return -EIO;			
			}

			/*Write 20h to MD1_CFG // Wake-up interrupt driven to INT1 pin */			
			register_value = LSM6DSL_MASK_MD1_CFG_INT1_WU; 
			LOG_DBG("SET LSM6DSL_MASK_MD1_CFG_INT1_WU 0x%x ", register_value);
			if (drv_data->hw_tf->write_data(dev,
						LSM6DSL_REG_MD1_CFG,
						&register_value, 1) < 0) {
				LOG_ERR("Could not Set WAKE_UP_DUR register");
				return -EIO;
			}


			//For threshold trigger
			drv_data->threshold_handler = handler;
			if (handler == NULL) {
				return 0;
			}	
			
			drv_data->data_threshold_trigger = trig;
			break;
		
		case SENSOR_TRIG_MOTION:		
			LOG_DBG("trigger type is: SENSOR_TRIG_MOTION");
			/* It uses the significant motion detection */

			/*Power off the device*/
			lsm6dsl_pm_action(dev, PM_DEVICE_ACTION_SUSPEND);

			/*Enable write access to register bank A and not B*/
			register_value = LSM6DSL_MASK_FUNC_CFG_EN;
			LOG_DBG("SET LSM6DSL_REG_FUNC_CFG_ACCESS 0x%x ", register_value);
			if (drv_data->hw_tf->write_data(dev,
						LSM6DSL_REG_FUNC_CFG_ACCESS,
						&register_value, 1) < 0) {
				LOG_ERR("Could not enable write function to bank A and B");
				return -EIO;			
			}

			/*Modify embedded function register in Bank A.All modif
			ications of the content of the embedded functions registers have to be performed
			with the device in power-down mode. */
			/*TODO: Modify register in bank	 A for sensing threshold*/
			// 			The embedded function register (accessible by setting the FUNC_CFG_EN bit of FUNC_CFG_ACCESS to 1)
			// used to configure the significant motion threshold parameter is the SM_THS register. The SM_THS_[7:0] bits of
			// this register define the threshold value: it corresponds to the number of steps to be performed by the user upon a
			// change of location before the significant motion interrupt is generated. It is expressed as an 8-bit unsigned value:
			// the default value of this field is equal to 6 (= 00000110b).
			register_value = 1;
			LOG_DBG("SET BANK_A SM_THS  0x%x ", register_value);
			if (drv_data->hw_tf->write_data(dev,
						LSM6DSL_BANK_A_SM_THS,
						&register_value, 1) < 0) {
				LOG_ERR("Could not write the value to LSM6DSL_BANK_A_SM_THS");
				return -EIO;			
			}

			/* Disable write access register for the Banks */
			register_value = 0;
			LOG_DBG("SET LSM6DSL_REG_FUNC_CFG_ACCESS 0x%x ", register_value);
			if (drv_data->hw_tf->write_data(dev,
						LSM6DSL_REG_FUNC_CFG_ACCESS,
						&register_value, 1) < 0) {
				LOG_ERR("Could not disable write function to bank A and B");
				return -EIO;			
			}

			/*TODO: Power on device*/
			lsm6dsl_pm_action(dev, PM_DEVICE_ACTION_RESUME);

			/* Enable embedded functionalities */
			register_value = LSM6DSL_MASK_CTRL10_C_FUNC_EN | LSM6DSL_MASK_CTRL10_C_SIGN_MOTION_EN;	
			LOG_DBG("SET CTRL10_C 0x%x ", register_value);
			if (drv_data->hw_tf->write_data(dev,
						LSM6DSL_REG_CTRL10_C,
						&register_value, 1) < 0) {
				LOG_ERR("Could not enable motion detection function");
				return -EIO;			
			}

			/* enable significant motion detection interrupt signal 1*/
			//update int1_ctrl  for INT1_SIGN_MOT  			
			register_value = LSM6DSL_MASK_INT1_CTRL_SIGN_MOT;	
			LOG_DBG("SET CTRL1 0x%x ", register_value);
			if (drv_data->hw_tf->write_data(dev,
						LSM6DSL_REG_INT1_CTRL,
						&register_value, 1) < 0) {
				LOG_ERR("Could not enable motion detection int1 interrupt.");
				return -EIO;			
			}
			break;
	
		//in case of data ready!
			//__ASSERT(trig->type == SENSOR_TRIG_DATA_READY, "It is of type SENSOR_TRIG_DATA_READY");
			// /* enable data-ready interrupt */
			// if (drv_data->hw_tf->update_reg(dev,
			// 		       LSM6DSL_REG_INT1_CTRL,
			// 		       LSM6DSL_MASK_INT1_CTRL_DRDY_XL |
			// 		       LSM6DSL_MASK_INT1_CTRL_DRDY_G,
			// 		       BIT(LSM6DSL_SHIFT_INT1_CTRL_DRDY_XL) |
			// 		       BIT(LSM6DSL_SHIFT_INT1_CTRL_DRDY_G)) < 0) {
			// 	LOG_ERR("Could not enable data-ready interrupt.");
			// 	return -EIO;
			// }
			
	default: 
		LOG_ERR("Trigger type not implemented");
		return -EIO;
	}

	/* If irq_gpio is not configured in DT just return error */
	if (!config->int_gpio.port){
		LOG_ERR("Triggers not supported");
		return -ENOTSUP;
	}

	setup_irq(dev, false);

	//for data ready trigger.
	//to be moved to the branching for this trigger type
	drv_data->data_ready_handler = handler;
	if (handler == NULL) {
		return 0;
	}
	drv_data->data_ready_trigger = trig;
	////


	setup_irq(dev, true);
	if (gpio_pin_get_dt(&config->int_gpio) > 0) {
		handle_irq(dev);
	}

	return 0;
}

static void lsm6dsl_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct lsm6dsl_data *drv_data =
		CONTAINER_OF(cb, struct lsm6dsl_data, gpio_cb);

	ARG_UNUSED(pins);
	LOG_DBG("GPIO callback. Interrupt triggered");

	handle_irq(drv_data->dev);
}

static void lsm6dsl_thread_cb(const struct device *dev)
{
	struct lsm6dsl_data *drv_data = dev->data;		
	if (1){
		//Determine what triggered the interrupt from the wakeup register
		uint8_t interrupt_registers_status[4]={0,0,0,0};
		if (drv_data->hw_tf->read_data(dev, LSM6DSL_REG_WAKE_UP_SRC, interrupt_registers_status, 4) < 0){
				LOG_WRN("Error while reading reg LSM6DSL_REG_WAKE_UP_SRC ");
			return;
		};
		LOG_DBG ("Trigger Status REG 0x1B caused by 0x%2x   0x%2x   0x%2x   0x%2x" , interrupt_registers_status[0],  interrupt_registers_status[1],  interrupt_registers_status[2],  interrupt_registers_status[3]);
		if (interrupt_registers_status[0] & LSM6DSL_MASK_WAKE_UP_SRC_FF_IA){
			LOG_DBG("Free fall detected");
		}

		if (interrupt_registers_status[0] & LSM6DSL_MASK_WAKE_UP_SRC_WU_IA){
			LOG_DBG("Wakeup event detected");
			if (interrupt_registers_status[0] & LSM6DSL_MASK_WAKE_UP_SRC_X_WU){
				LOG_DBG("X threshold exceeded");
			}
			if (interrupt_registers_status[0] & LSM6DSL_MASK_WAKE_UP_SRC_Y_WU){
				LOG_DBG("Y threshold exceeded");
			}	
			if (interrupt_registers_status[0] & LSM6DSL_MASK_WAKE_UP_SRC_Z_WU){
				LOG_DBG("Z threshold exceeded");
			}
			//read the data from the device
			sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);

			if (drv_data->threshold_handler != NULL) {
				drv_data->threshold_handler(dev, drv_data->data_threshold_trigger);
			}

			setup_irq(dev, true);
			return;
		}

		if (interrupt_registers_status[0] & LSM6DSL_MASK_WAKE_UP_SRC_WU_IA){
			//LOG_DBG("Sleep state detected");
		}

		if (0){
			uint8_t FUNC_SRC_value[2]={0xFF,0xFF};
			if (drv_data->hw_tf->read_data(dev, LSM6DSL_REG_FUNC_SRC1, FUNC_SRC_value,2) < 0){
				LOG_WRN("Error while reading reg FUNC_SRC1");
				return;
			};
			LOG_DBG ("Trigger int1 caused by 0x%x   0x%x" , FUNC_SRC_value[0],  FUNC_SRC_value[1]);
			if (FUNC_SRC_value[0] & LSM6DSL_MASK_FUNC_SRC1_SIGN_MOTION_IA){
				LOG_DBG("Significant motion detected");
			}
			else{
				LOG_DBG("Significant motion not detected");
			}
		}
	}

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev, drv_data->data_ready_trigger);
	}

	setup_irq(dev, true);
}

#ifdef CONFIG_LSM6DSL_TRIGGER_OWN_THREAD
static void lsm6dsl_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct lsm6dsl_data *drv_data = dev->data;

	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		lsm6dsl_thread_cb(dev);
	}
}
#endif

#ifdef CONFIG_LSM6DSL_TRIGGER_GLOBAL_THREAD
static void lsm6dsl_work_cb(struct k_work *work)
{
	struct lsm6dsl_data *drv_data =
		CONTAINER_OF(work, struct lsm6dsl_data, work);

	lsm6dsl_thread_cb(drv_data->dev);
}
#endif

int lsm6dsl_init_interrupt(const struct device *dev)
{
	const struct lsm6dsl_config *config = dev->config;
	struct lsm6dsl_data *drv_data = dev->data;

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);

	gpio_init_callback(&drv_data->gpio_cb,
			   lsm6dsl_gpio_callback, BIT(config->int_gpio.pin));

	if (gpio_add_callback(config->int_gpio.port, &drv_data->gpio_cb) < 0) {
		LOG_ERR("Could not set gpio callback.");
		return -EIO;
	}

	drv_data->dev = dev;

#if defined(CONFIG_LSM6DSL_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_LSM6DSL_THREAD_STACK_SIZE,
			lsm6dsl_thread, (void *)dev,
			NULL, NULL, K_PRIO_COOP(CONFIG_LSM6DSL_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_LSM6DSL_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = lsm6dsl_work_cb;
#endif

	setup_irq(dev, true);

	return 0;
}
