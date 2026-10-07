/** @file
 *  @brief Host-test shim for <driver/i2c_master.h> (V2.8.8).
 *
 *  Opaque handle types only: headers such as pm_sensor.h name them in
 *  prototypes, and a host test that includes those headers never calls the
 *  I2C driver. See test/shim/README.md.
 */
#pragma once

typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;
typedef struct i2c_master_dev_t *i2c_master_dev_handle_t;
