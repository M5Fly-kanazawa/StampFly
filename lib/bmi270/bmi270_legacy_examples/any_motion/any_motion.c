/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi270_legacy.h"
#include "common.h"

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for any-motion.
 *
 *  @param[in] bmi2_dev       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_feature_config(struct bmi2_dev *bmi2_dev);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Sensor initialization configuration. */
    struct bmi2_dev dev;

    /* Accel sensor and any-motion feature are listed in array. */
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_ANY_MOTION };

    /* Variable to get any-motion interrupt status. */
    uint16_t int_status = 0;

    /* Select features and their pins to be mapped to. */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_ANY_MOTION, .hw_int_pin = BMI2_INT1 };

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&dev, BMI2_SPI_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270_legacy. */
    rslt = bmi270_legacy_init(&dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* Enable the selected sensors. */
        rslt = bmi270_legacy_sensor_enable(sens_list, 2, &dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK)
        {
            /* Set feature configurations for any-motion. */
            rslt = set_feature_config(&dev);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                /* Map the feature interrupt for any-motion. */
                rslt = bmi270_legacy_map_feat_int(&sens_int, 1, &dev);
                bmi2_error_codes_print_result(rslt);

                printf("Move the board in any direction\n");

                /* Loop to print the any-motion interrupt status. */
                do
                {
                    /* Clear the buffer. */
                    int_status = 0;

                    /* To get the interrupt status of any-motion. */
                    rslt = bmi2_get_int_status(&int_status, &dev);
                    bmi2_error_codes_print_result(rslt);

                    /* To check the interrupt status of any-motion. */
                    if (int_status & BMI270_LEGACY_ANY_MOT_STATUS_MASK)
                    {
                        printf("Any-motion interrupt is generated\n");
                        break;
                    }
                } while (rslt == BMI2_OK);
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for any-motion.
 */
static int8_t set_feature_config(struct bmi2_dev *bmi2_dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Configure the type of any-motion feature. */
    config.type = BMI2_ANY_MOTION;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_legacy_get_sensor_config(&config, 1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* 1LSB equals 20ms. Default is 100ms, setting to 80ms. */
        config.cfg.any_motion.duration = 0x04;

        /* 1LSB equals to 0.48mg. Default is 83mg, setting to 50mg. */
        config.cfg.any_motion.threshold = 0x68;

        /* Set new configurations. */
        rslt = bmi270_legacy_set_sensor_config(&config, 1, bmi2_dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}
