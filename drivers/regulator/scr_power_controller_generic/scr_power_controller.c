
#define DT_DRV_COMPAT scr_power_controller_generic

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <stdint.h>

LOG_MODULE_REGISTER(scr_power_controller_generic, CONFIG_REGULATOR_LOG_LEVEL);

struct scr_power_controller_generic_config {
  struct regulator_common_config common;
  struct gpio_dt_spec zero_crossing_gpio;
  struct gpio_dt_spec enable_gpio;
};

struct fraction {
  uint32_t numerator;
  uint32_t denominator;
};

struct psm_config {
  // Device will be enabled for on_pulses pulses out of period (=
  // on_pulses/period of the time)
  struct fraction ratio;
  struct fraction actual_ratio;
  int distribute;
};

struct scr_power_controller_generic_data {
  struct regulator_common_data common;
  const struct device *dev;
  struct gpio_callback zc_cb;
  struct psm_config psm_cfg;
  uint32_t zc_counter;
  int enable;
};

static int scr_power_controller_generic_enable(const struct device *dev);
static int scr_power_controller_generic_disable(const struct device *dev);
static int scr_power_controller_generic_is_enabled(const struct device *dev);
static void scr_power_controller_set_pin_for_counter(const struct device *dev, struct fraction ratio, uint32_t counter);

uint32_t calc_gcd(uint32_t n1, uint32_t n2) {
  if (!n1 || !n2)
    return 0;

  while (n1 != n2) {
    if (n1 > n2)
      n1 -= n2;
    else
      n2 -= n1;
  }

  return n1;
}

int scr_power_controller_generic_set_psm(const struct device *dev,
                                         uint32_t on_pulses, uint32_t period) {
  if (on_pulses > period)
    return -1;

  if (on_pulses == 0 && scr_power_controller_generic_is_enabled(dev)) {
    scr_power_controller_generic_disable(dev);
  }

  struct scr_power_controller_generic_data *data = dev->data;
  data->psm_cfg.ratio.denominator = period;
  data->psm_cfg.actual_ratio.denominator = period;
  data->psm_cfg.ratio.numerator = on_pulses;
  data->psm_cfg.actual_ratio.numerator = on_pulses;

  if (data->psm_cfg.distribute) {
    uint32_t gcd = calc_gcd(period, on_pulses);

    if (gcd > 1) {
      data->psm_cfg.actual_ratio.denominator /= gcd;
      data->psm_cfg.actual_ratio.numerator /= gcd;
    }
  }

  // TODO: should this be reset?
  data->zc_counter = 0;

  if (on_pulses > 0 && !scr_power_controller_generic_is_enabled(dev)) {
    scr_power_controller_generic_enable(dev);
  }

  if (scr_power_controller_generic_is_enabled(dev))
    scr_power_controller_set_pin_for_counter(dev, data->psm_cfg.actual_ratio, data->zc_counter);

  return 0;
}

int scr_power_controller_set_cycles(const struct device *dev, uint32_t channel,
                                    uint32_t period, uint32_t pulse,
                                    pwm_flags_t flags) {
  LOG_DBG("Set %s channel %u cycles: period %u, pulse: %u", dev->name, channel, period, pulse);
  return scr_power_controller_generic_set_psm(dev, pulse, period);
}

int scr_power_controller_generic_set_enable_pin(const struct device *dev,
                                                int enable) {
  LOG_DBG("Set %s enable pin: %d", dev->name, enable);
  const struct scr_power_controller_generic_config *cfg = dev->config;
  return gpio_pin_set_dt(&cfg->enable_gpio, enable);
}

static void scr_power_controller_set_pin_for_counter(const struct device *dev, struct fraction ratio, uint32_t counter) {
  int pass_power_pulse = (counter < ratio.numerator);
  scr_power_controller_generic_set_enable_pin(dev, pass_power_pulse);
}

static void scr_power_controller_generic_zc_interrupt(
    const struct device *dev, struct gpio_callback *cbdata, uint32_t pins) {
  struct scr_power_controller_generic_data *data =
      CONTAINER_OF(cbdata, struct scr_power_controller_generic_data, zc_cb);
  const struct scr_power_controller_generic_config *cfg = data->dev->config;

  uint32_t count = data->zc_counter;

  if (data->psm_cfg.actual_ratio.denominator) {
    ++data->zc_counter;
    // TODO: should this be reset? -> always count up, calc modulo for temp var
    // instead
    data->zc_counter %= data->psm_cfg.actual_ratio.denominator;
  }

  int pass_power_pulse =
      (count < data->psm_cfg.actual_ratio.numerator) && data->enable;

  // bool is_enabled = regulator_is_enabled(dev);
  bool is_enabled = data->enable;
  if (is_enabled) {
    LOG_DBG(
        "SCR power controller zero crossing %" PRIu32 " | %" PRIu32 "/%" PRIu32
        " == %" PRIu32 "/%" PRIu32 " detected at %" PRIu32 " => %d",
        count, data->psm_cfg.ratio.numerator, data->psm_cfg.ratio.denominator,
        data->psm_cfg.actual_ratio.numerator,
        data->psm_cfg.actual_ratio.denominator, k_cycle_get_32(),
        pass_power_pulse);

    scr_power_controller_set_pin_for_counter(data->dev, data->psm_cfg.actual_ratio, count);
  } else {
    scr_power_controller_generic_set_enable_pin(data->dev, 0);
  }
}

static int scr_power_controller_generic_enable(const struct device *dev) {
  const struct scr_power_controller_generic_config *cfg = dev->config;
  struct scr_power_controller_generic_data *data = dev->data;
  int ret;

  LOG_DBG("Enable regulator: %s", dev->name);

  if (!cfg->enable_gpio.port) {
    return -ENOTSUP;
  }

  data->enable = 1;

  ret = gpio_pin_interrupt_configure_dt(&cfg->zero_crossing_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Zero crossing gpio interrupt enable failed: %d", ret);
    return ret;
  }

  // enable pin will be set in next zero crossing interrupt

  // ret = gpio_pin_set_dt(&cfg->enable_gpio, 1);
  // if (ret < 0) {
  //   return ret;
  // }

  return 0;
}

static int scr_power_controller_generic_is_enabled(const struct device *dev) {
  const struct scr_power_controller_generic_config *cfg = dev->config;
  struct scr_power_controller_generic_data *data = dev->data;
  return data->enable;
}

static int scr_power_controller_generic_disable(const struct device *dev) {
  const struct scr_power_controller_generic_config *cfg = dev->config;
  struct scr_power_controller_generic_data *data = dev->data;

  LOG_DBG("Disable regulator: %s", dev->name);

  if (!cfg->enable_gpio.port) {
    return -ENOTSUP;
  }

  data->enable = 0;

  int ret = gpio_pin_interrupt_configure_dt(&cfg->zero_crossing_gpio,
                                            GPIO_INT_DISABLE);
  if (ret < 0) {
    LOG_ERR("Zero crossing gpio interrupt disable failed: %d", ret);
    return ret;
  }

  // TODO: turn off scr and disable activation in interrupt
  // return gpio_pin_set_dt(&cfg->enable_gpio, 0);
  return scr_power_controller_generic_set_enable_pin(dev, 0);
}

static int scr_power_controller_generic_init(const struct device *dev) {
  const struct scr_power_controller_generic_config *cfg = dev->config;
  struct scr_power_controller_generic_data *data = dev->data;
  int ret;

  // regulator_common_data_init(dev);

  data->dev = dev;

  // TODO: for debug
  // scr_power_controller_generic_set_psm(dev, 4, 6);

  const struct gpio_dt_spec *gpio_zc = &cfg->zero_crossing_gpio;
  const struct gpio_dt_spec *enable_gpio = &cfg->enable_gpio;

  if (!gpio_is_ready_dt(gpio_zc)) {
    LOG_ERR("%s is not ready", gpio_zc->port->name);
    return -ENODEV;
  }

  if (!gpio_is_ready_dt(enable_gpio)) {
    LOG_ERR("%s is not ready", enable_gpio->port->name);
    return -ENODEV;
  }

  ret = gpio_pin_configure_dt(gpio_zc, GPIO_INPUT);
  if (ret != 0) {
    LOG_ERR("Zero crossing pin configuration failed: %d", ret);
    return ret;
  }

  ret = gpio_pin_configure_dt(enable_gpio, GPIO_OUTPUT_INACTIVE);
  if (ret != 0) {
    LOG_ERR("Enable pin configuration failed: %d", ret);
    return ret;
  }

  gpio_init_callback(&data->zc_cb, scr_power_controller_generic_zc_interrupt,
                     BIT(gpio_zc->pin));

  ret = gpio_add_callback(gpio_zc->port, &data->zc_cb);
  if (ret < 0) {
    LOG_ERR("Could not set zero crossing callback");
    return ret;
  }

  ret = gpio_pin_interrupt_configure_dt(gpio_zc, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Zero crossing gpio interrupt configuration failed: %d", ret);
    return ret;
  }

  // return regulator_common_init(dev, false);
  return 0;
}

static const struct regulator_driver_api scr_power_controller_generic_api = {
    .enable = scr_power_controller_generic_enable,
    .disable = scr_power_controller_generic_disable,
    // .count_voltages = scr_power_controller_generic_count_voltages,
    // .list_voltage = scr_power_controller_generic_list_voltage,
};

static const struct pwm_driver_api scr_power_controller_pwm_api = {
    .set_cycles = scr_power_controller_set_cycles,
    // .get_cycles_per_sec = imx_pwm_get_cycles_per_sec,
};

#ifdef CONFIG_PM_DEVICE
static int
scr_power_controller_generic_pm_action(const struct device *dev,
                                       enum pm_device_action action) {
  switch (action) {
  case PM_DEVICE_ACTION_TURN_ON:
  case PM_DEVICE_ACTION_RESUME:
  case PM_DEVICE_ACTION_TURN_OFF:
  case PM_DEVICE_ACTION_SUSPEND:
  default:
    return -ENOTSUP;
  }
}
#endif

#define SCR_POWER_CONTROLLER_GENERIC_DEFINE(i)                                 \
                                                                               \
  static const struct scr_power_controller_generic_config                      \
      scr_power_controller_generic_config_##i = {                              \
          .common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(i),                   \
          .zero_crossing_gpio = GPIO_DT_SPEC_INST_GET(i, zero_crossing_gpios), \
          .enable_gpio = GPIO_DT_SPEC_INST_GET(i, enable_gpios),               \
  };                                                                           \
                                                                               \
  static struct scr_power_controller_generic_data                              \
      scr_power_controller_generic_data_##i = {                                \
          .psm_cfg =                                                           \
              {                                                                \
                  .ratio = {.denominator = 0, .numerator = 0},                 \
                  .distribute = 0,                                             \
                  .actual_ratio = {.denominator = 0, .numerator = 0},          \
              },                                                               \
          .zc_counter = 0,                                                     \
          .enable = 0,                                                         \
  };                                                                           \
                                                                               \
  PM_DEVICE_DT_INST_DEFINE(i, scr_power_controller_generic_pm_action);         \
                                                                               \
  DEVICE_DT_INST_DEFINE(                                                       \
      i, &scr_power_controller_generic_init, PM_DEVICE_DT_INST_GET(i),         \
      &scr_power_controller_generic_data_##i,                                  \
      &scr_power_controller_generic_config_##i, POST_KERNEL,                   \
      CONFIG_INPUT_INIT_PRIORITY, &scr_power_controller_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(SCR_POWER_CONTROLLER_GENERIC_DEFINE)
