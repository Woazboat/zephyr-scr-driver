# Zephyr - SCR Driver

Zephyr RTOS driver for silicone controlled rectifiers with zero crossing detection + pulse skip modulation (PSM).


Warning: Currently half baked and completely unpolished / only partially complete. Use at your own risk.

Kconfig:

```conf
CONFIG_REGULATOR=y
```

Devicetree:

```devicetree
example_scr: scr_power_controller_0 {
		compatible = "scr-power-controller-generic";
		regulator-name = "Example SCR";
		enable-gpios = <&gpio0 6 (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH)>;
		zero-crossing-gpios = <&gpio0 5 (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH)>;
		status = "okay";
		// regulator-always-on;
	};
```

Usage:

```cpp
const struct device *const example_scr_dev = DEVICE_DT_GET(DT_NODELABEL(example_scr));

int main()
{
    if (!device_is_ready(example_scr_dev)) {
        LOG_ERR("Device not ready: %s", example_scr_dev->name);
        return -ENODEV;
    }

    // Using the PWM API
    LOG_DBG("Set SCR pulse to 4/6");
    if (int ret = pwm_set_cycles(example_scr_dev, 0, 6, 4, {}); ret != 0) {
        LOG_ERR("Error: unable to configure SCR: %d", ret);
        return -1;
    }
}
```
