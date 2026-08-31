#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/bool.h>

extern "C" {
    #include "pico_uart_transports.h"
}

// ============================================================
// PIN MAP
// ============================================================

// INA226 - I2C1
constexpr uint VL_SDA_PIN  = 2;
constexpr uint VL_SCL_PIN  = 3;
constexpr uint VL_ALRT_PIN = 6;

// WCS1800
constexpr uint CR_AOUT_PIN = 26;  // ADC0
constexpr uint CR_DOUT_PIN = 22;

// Buzzer
constexpr uint BUZZER_PIN = 15;

// Upper Controller / micro-ROS UART0
constexpr uint UART0_TX_PIN = 0;
constexpr uint UART0_RX_PIN = 1;

// Motor Pico UART1
constexpr uint UART1_TX_PIN = 4;
constexpr uint UART1_RX_PIN = 5;

// ============================================================
// UART / I2C
// ============================================================

constexpr uint32_t UPPER_UART_BAUD = 115200;
constexpr uint32_t MOTOR_UART_BAUD = 115200;

constexpr uint32_t INA226_I2C_BAUD = 400000;

// INA226 default address:
// A0 = GND, A1 = GND -> 0x40
constexpr uint8_t INA226_ADDR = 0x40;

// ============================================================
// BATTERY
// ============================================================

constexpr float BATTERY_CAPACITY_AH = 19.2f;

constexpr float BATTERY_FULL_V    = 29.40f;
constexpr float BATTERY_NOMINAL_V = 25.84f;
constexpr float BATTERY_CUTOFF_V  = 21.00f;

constexpr float SOC_WARNING_PERCENT = 15.0f;
constexpr float SOC_STOP_PERCENT    = 10.0f;

// 저전압이 순간적으로 한번 찍혔다고 바로 정지하지 않도록
constexpr uint32_t LOW_VOLTAGE_CONFIRM_MS = 500;

// ============================================================
// WCS1800
// ============================================================

constexpr float ADC_REF_V = 3.3f;
constexpr float ADC_MAX   = 4095.0f;

/*
 * 중요:
 *
 * 이 값은 실제 3.3 V 구동 WCS 센서를
 * 알려진 부하 전류로 실측한 뒤 수정.
 *
 * 예:
 * sensitivity = abs(Vload - Vzero) / known_current
 *
 * 아래 0.050 V/A는 임시값.
 */

constexpr float WCS_SENSITIVITY_V_PER_A = 0.050f;

constexpr float WCS_CURRENT_GAIN = 1.000f;

// +/- 이 이하의 값은 0A 처리
constexpr float CURRENT_DEADBAND_A = 0.10f;

// 저역통과 필터
constexpr float CURRENT_FILTER_ALPHA = 0.05f;

// 부팅 영점 샘플
constexpr uint32_t ZERO_CAL_SAMPLES = 2000;
constexpr uint32_t ZERO_CAL_DELAY_US = 250;

// ============================================================
// TIMING
// ============================================================

// Core1 power loop
constexpr uint32_t POWER_LOOP_MS = 10;       // 100Hz

// INA226
constexpr uint32_t VOLTAGE_READ_MS = 100;    // 10Hz

// ROS publish
constexpr uint32_t ROS_PUBLISH_MS = 200;     // 5Hz

// ============================================================
// INA226 REGISTERS
// ============================================================

constexpr uint8_t INA226_REG_CONFIG      = 0x00;
constexpr uint8_t INA226_REG_BUS_VOLTAGE = 0x02;
constexpr uint8_t INA226_REG_MASK_ENABLE = 0x06;

// INA226 Bus Voltage LSB = 1.25mV
constexpr float INA226_BUS_LSB_V = 0.00125f;

// ============================================================
// STATE
// ============================================================

enum class PowerState : uint8_t
{
    STARTUP = 0,
    RUNNING,
    LOW_BATTERY,
    CRITICAL_BATTERY,
    FAULT
};

enum FaultFlags : uint32_t
{
    FAULT_NONE             = 0,
    FAULT_LOW_SOC          = 1u << 0,
    FAULT_LOW_VOLTAGE      = 1u << 1,
    FAULT_INA226           = 1u << 2,
    FAULT_CURRENT_SENSOR   = 1u << 3,
    FAULT_CURRENT_DIGITAL  = 1u << 4
};

struct PowerStatus
{
    float voltage_v;

    // 양수 = 배터리에서 소비되는 전류
    float current_a;

    float consumed_ah;
    float remaining_ah;
    float soc_percent;

    float current_zero_v;

    uint16_t raw_current_adc;

    PowerState state;
    uint32_t fault_flags;

    bool low_battery;
    bool critical_battery;
    bool motor_run_allowed;

    uint32_t heartbeat;
};

// ============================================================
// SHARED STATE
// ============================================================

static PowerStatus g_status{};
static critical_section_t g_status_lock;

static volatile bool g_core1_ready = false;

// ============================================================
// ROS
// ============================================================

static rcl_publisher_t g_battery_pub;
static rcl_publisher_t g_fault_pub;
static rcl_publisher_t g_alert_ack_pub;
static rcl_subscription_t g_alert_ok_sub;

static sensor_msgs__msg__BatteryState g_battery_msg;
static std_msgs__msg__UInt32 g_fault_msg;
static std_msgs__msg__Bool g_alert_ok_msg;
static std_msgs__msg__Bool g_alert_ack_msg;

static volatile bool g_ready_chime_request = false;
static volatile bool g_ready_sequence_completed = false;
static volatile bool g_alert_ack_request = false;

// ============================================================
// UTILITY
// ============================================================

static inline float clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

// ============================================================
// BUZZER PWM
// ============================================================

constexpr uint32_t BUZZER_BOOT_FREQ_HZ  = 2000;
constexpr uint32_t BUZZER_FAULT_FREQ_HZ = 1800;
constexpr uint32_t BUZZER_WARN_FREQ_HZ  = 1800;

// CUBIC system READY melody: C5 -> E5 -> G5
constexpr uint32_t BUZZER_READY_C_HZ = 523;
constexpr uint32_t BUZZER_READY_E_HZ = 659;
constexpr uint32_t BUZZER_READY_G_HZ = 784;

constexpr uint16_t BUZZER_PWM_WRAP = 1000;

static void buzzer_init()
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel = pwm_gpio_to_channel(BUZZER_PIN);

    pwm_set_wrap(slice, BUZZER_PWM_WRAP);
    pwm_set_chan_level(slice, channel, 0);
    pwm_set_enabled(slice, false);
}

static void buzzer_on(uint32_t frequency_hz)
{
    if (frequency_hz == 0)
        return;

    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel = pwm_gpio_to_channel(BUZZER_PIN);

    uint32_t sys_clk_hz = clock_get_hz(clk_sys);

    float clkdiv =
        static_cast<float>(sys_clk_hz) /
        (
            static_cast<float>(frequency_hz) *
            static_cast<float>(BUZZER_PWM_WRAP + 1)
        );

    // RP2350 PWM divider valid range 보호
    if (clkdiv < 1.0f)
        clkdiv = 1.0f;

    if (clkdiv > 255.9375f)
        clkdiv = 255.9375f;

    pwm_set_enabled(slice, false);
    pwm_set_clkdiv(slice, clkdiv);
    pwm_set_wrap(slice, BUZZER_PWM_WRAP);

    // 50% duty
    pwm_set_chan_level(
        slice,
        channel,
        BUZZER_PWM_WRAP / 2
    );

    pwm_set_enabled(slice, true);
}

static void buzzer_off()
{
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel = pwm_gpio_to_channel(BUZZER_PIN);

    pwm_set_chan_level(slice, channel, 0);
    pwm_set_enabled(slice, false);
}

static void buzzer_tone(
    uint32_t frequency_hz,
    uint32_t duration_ms)
{
    buzzer_on(frequency_hz);
    sleep_ms(duration_ms);
    buzzer_off();
}

static void boot_beep_ok()
{
    buzzer_tone(
        BUZZER_BOOT_FREQ_HZ,
        150
    );
}

static void boot_beep_fault()
{
    for (int i = 0; i < 3; ++i)
    {
        buzzer_tone(
            BUZZER_FAULT_FREQ_HZ,
            100
        );

        sleep_ms(100);
    }
}

static void ready_chime()
{
    buzzer_tone(BUZZER_READY_C_HZ, 150);
    sleep_ms(40);

    buzzer_tone(BUZZER_READY_E_HZ, 150);
    sleep_ms(40);

    buzzer_tone(BUZZER_READY_G_HZ, 250);
}

static void alert_ok_callback(const void *msgin)
{
    const std_msgs__msg__Bool *msg =
        static_cast<const std_msgs__msg__Bool *>(msgin);

    if (!msg->data)
        return;

    // READY melody is one-shot per PCU boot.
    //
    // If SBC retransmits alert_ok because it did not receive
    // the ACK, do NOT replay the melody. Just send ACK again.
    if (g_ready_sequence_completed)
    {
        g_alert_ack_request = true;
        return;
    }

    // Do not block inside the micro-ROS callback.
    // Main loop plays the melody and then requests ACK.
    g_ready_chime_request = true;
}

// ============================================================
// MOTOR PICO UART
// ============================================================

static void motor_send(const char *cmd)
{
    uart_puts(uart1, cmd);
}

static void motor_stop()
{
    motor_send("STOP\n");
}

static void motor_run()
{
    motor_send("RUN\n");
}

// ============================================================
// INA226
// ============================================================

static bool ina226_read_register(uint8_t reg, uint16_t &value)
{
    int ret = i2c_write_blocking(
        i2c1,
        INA226_ADDR,
        &reg,
        1,
        true
    );

    if (ret != 1)
        return false;

    uint8_t data[2];

    ret = i2c_read_blocking(
        i2c1,
        INA226_ADDR,
        data,
        2,
        false
    );

    if (ret != 2)
        return false;

    value =
        (static_cast<uint16_t>(data[0]) << 8) |
        static_cast<uint16_t>(data[1]);

    return true;
}

static bool ina226_write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[3];

    data[0] = reg;
    data[1] = static_cast<uint8_t>(value >> 8);
    data[2] = static_cast<uint8_t>(value & 0xFF);

    return i2c_write_blocking(
        i2c1,
        INA226_ADDR,
        data,
        3,
        false
    ) == 3;
}

static bool ina226_init()
{
    constexpr uint16_t config = 0x4527;

    if (!ina226_write_register(INA226_REG_CONFIG, config))
        return false;

    sleep_ms(5);

    uint16_t test;

    if (!ina226_read_register(INA226_REG_BUS_VOLTAGE, test))
        return false;

    return true;
}

static bool ina226_read_voltage(float &voltage)
{
    uint16_t raw;

    if (!ina226_read_register(INA226_REG_BUS_VOLTAGE, raw))
        return false;

    voltage = static_cast<float>(raw) * INA226_BUS_LSB_V;

    return true;
}

// ============================================================
// WCS1800
// ============================================================

static uint16_t current_adc_read()
{
    adc_select_input(0); // GPIO26 = ADC0
    return adc_read();
}

static float adc_to_voltage(uint16_t adc)
{
    return
        static_cast<float>(adc) *
        ADC_REF_V /
        ADC_MAX;
}

static float calibrate_current_zero()
{
    uint64_t sum = 0;

    for (uint32_t i = 0; i < ZERO_CAL_SAMPLES; ++i)
    {
        sum += current_adc_read();
        sleep_us(ZERO_CAL_DELAY_US);
    }

    float average =
        static_cast<float>(sum) /
        static_cast<float>(ZERO_CAL_SAMPLES);

    return adc_to_voltage(
        static_cast<uint16_t>(average)
    );
}

static float current_from_voltage(
    float sensor_v,
    float zero_v)
{
    float current =
        (sensor_v - zero_v) /
        WCS_SENSITIVITY_V_PER_A;

    current *= WCS_CURRENT_GAIN;

    if (fabsf(current) < CURRENT_DEADBAND_A)
        current = 0.0f;

    return current;
}

// ============================================================
// INITIAL SOC
// ============================================================

/*
 * 현재 배터리에서 확정된 값:
 *
 * 29.40 V = full
 * 25.84 V = nominal
 * 21.00 V = cutoff
 *
 * 정확한 cell OCV 곡선은 아직 없으므로 초기 SOC만
 * 두 구간으로 근사.
 *
 * 이후 SOC는 WCS 전류적산이 담당.
 */
static float voltage_to_initial_soc(float voltage)
{
    if (voltage >= BATTERY_FULL_V)
        return 100.0f;

    if (voltage <= BATTERY_CUTOFF_V)
        return 0.0f;

    if (voltage >= BATTERY_NOMINAL_V)
    {
        float ratio =
            (voltage - BATTERY_NOMINAL_V) /
            (BATTERY_FULL_V - BATTERY_NOMINAL_V);

        return
            50.0f +
            ratio * 50.0f;
    }
    else
    {
        float ratio =
            (voltage - BATTERY_CUTOFF_V) /
            (BATTERY_NOMINAL_V - BATTERY_CUTOFF_V);

        return ratio * 50.0f;
    }
}

// ============================================================
// SHARED STATUS
// ============================================================

static void write_shared_status(const PowerStatus &status)
{
    critical_section_enter_blocking(&g_status_lock);
    g_status = status;
    critical_section_exit(&g_status_lock);
}

static PowerStatus read_shared_status()
{
    PowerStatus copy;

    critical_section_enter_blocking(&g_status_lock);
    copy = g_status;
    critical_section_exit(&g_status_lock);

    return copy;
}

// ============================================================
// CORE 1
// ============================================================

static void core1_main()
{
    PowerStatus status{};

    status.state = PowerState::STARTUP;
    status.fault_flags = FAULT_NONE;
    status.motor_run_allowed = false;

    // --------------------------------------------------------
    // ADC
    // --------------------------------------------------------

    adc_init();
    adc_gpio_init(CR_AOUT_PIN);
    adc_select_input(0);

    gpio_init(CR_DOUT_PIN);
    gpio_set_dir(CR_DOUT_PIN, GPIO_IN);

    // --------------------------------------------------------
    // INA226
    // --------------------------------------------------------

    bool ina_ok = ina226_init();

    if (!ina_ok)
    {
        status.fault_flags |= FAULT_INA226;
        status.state = PowerState::FAULT;

        write_shared_status(status);

        g_core1_ready = true;

        while (true)
        {
            status.heartbeat++;
            write_shared_status(status);
            sleep_ms(100);
        }
    }

    // --------------------------------------------------------
    // WCS zero calibration
    // --------------------------------------------------------

    status.current_zero_v = calibrate_current_zero();
    if (
        status.current_zero_v < 0.5f ||
        status.current_zero_v > 2.8f
    )
    {
        status.fault_flags |= FAULT_CURRENT_SENSOR;
        status.state = PowerState::FAULT;

        write_shared_status(status);

        g_core1_ready = true;

        while (true)
        {
            status.heartbeat++;
            write_shared_status(status);
            sleep_ms(100);
        }
    }

    // --------------------------------------------------------
    // Initial battery voltage
    // --------------------------------------------------------

    float initial_voltage = 0.0f;

    if (!ina226_read_voltage(initial_voltage))
    {
        status.fault_flags |= FAULT_INA226;
        status.state = PowerState::FAULT;

        write_shared_status(status);

        g_core1_ready = true;

        while (true)
        {
            status.heartbeat++;
            write_shared_status(status);
            sleep_ms(100);
        }
    }

    status.voltage_v = initial_voltage;

    status.soc_percent =
        voltage_to_initial_soc(initial_voltage);

    status.remaining_ah =
        BATTERY_CAPACITY_AH *
        status.soc_percent /
        100.0f;

    status.consumed_ah =
        BATTERY_CAPACITY_AH -
        status.remaining_ah;

    // --------------------------------------------------------
    // Startup safety
    // --------------------------------------------------------

    if (initial_voltage <= BATTERY_CUTOFF_V)
    {
        status.fault_flags |= FAULT_LOW_VOLTAGE;
        status.critical_battery = true;
        status.motor_run_allowed = false;
        status.state = PowerState::CRITICAL_BATTERY;
    }
    else if (status.soc_percent <= SOC_STOP_PERCENT)
    {
        status.fault_flags |= FAULT_LOW_SOC;
        status.critical_battery = true;
        status.motor_run_allowed = false;
        status.state = PowerState::CRITICAL_BATTERY;
    }
    else
    {
        status.motor_run_allowed = true;

        if (status.soc_percent <= SOC_WARNING_PERCENT)
        {
            status.low_battery = true;
            status.state = PowerState::LOW_BATTERY;
        }
        else
        {
            status.state = PowerState::RUNNING;
        }
    }

    write_shared_status(status);

    g_core1_ready = true;

    // --------------------------------------------------------
    // Main Core1 loop
    // --------------------------------------------------------

    uint64_t last_loop_us = time_us_64();
    uint64_t last_voltage_us = 0;

    uint64_t low_voltage_since_us = 0;

    float filtered_current = 0.0f;

    while (true)
    {
        uint64_t now_us = time_us_64();

        float dt =
            static_cast<float>(now_us - last_loop_us) /
            1000000.0f;

        last_loop_us = now_us;

        // ----------------------------------------------------
        // Current sampling
        // ----------------------------------------------------

        uint16_t raw = current_adc_read();
        float sensor_v = adc_to_voltage(raw);

        float current =
            current_from_voltage(
                sensor_v,
                status.current_zero_v
            );

        filtered_current =
            filtered_current +
            CURRENT_FILTER_ALPHA *
            (current - filtered_current);

        if (fabsf(filtered_current) < CURRENT_DEADBAND_A)
            filtered_current = 0.0f;

        status.raw_current_adc = raw;
        status.current_a = filtered_current;

        // ----------------------------------------------------
        // Coulomb counting
        // ----------------------------------------------------

        /*
         * 현재 시스템은 회생충전X.
         * 소비 방향의 전류만 적산.
         */
        if (filtered_current > 0.0f)
        {
            float delta_ah =
                filtered_current *
                dt /
                3600.0f;

            status.consumed_ah += delta_ah;
            status.remaining_ah -= delta_ah;

            status.remaining_ah =
                clampf(
                    status.remaining_ah,
                    0.0f,
                    BATTERY_CAPACITY_AH
                );

            status.consumed_ah =
                clampf(
                    status.consumed_ah,
                    0.0f,
                    BATTERY_CAPACITY_AH
                );

            status.soc_percent =
                status.remaining_ah /
                BATTERY_CAPACITY_AH *
                100.0f;
        }

        // ----------------------------------------------------
        // INA226 voltage 10Hz
        // ----------------------------------------------------

        if (
            now_us - last_voltage_us >=
            VOLTAGE_READ_MS * 1000ULL
        )
        {
            last_voltage_us = now_us;

            float voltage;

            if (ina226_read_voltage(voltage))
            {
                status.voltage_v = voltage;

                status.fault_flags &=
                    ~FAULT_INA226;
            }
            else
            {
                status.fault_flags |=
                    FAULT_INA226;
            }
        }

        // ----------------------------------------------------
        // WCS DOUT
        // ----------------------------------------------------


        bool current_digital = gpio_get(CR_DOUT_PIN);

        // ----------------------------------------------------
        // LOW VOLTAGE debounce
        // ----------------------------------------------------

        bool voltage_critical = false;

        if (status.voltage_v <= BATTERY_CUTOFF_V)
        {
            if (low_voltage_since_us == 0)
            {
                low_voltage_since_us = now_us;
            }

            if (
                now_us - low_voltage_since_us >=
                LOW_VOLTAGE_CONFIRM_MS * 1000ULL
            )
            {
                voltage_critical = true;
            }
        }
        else
        {
            low_voltage_since_us = 0;
        }

        // ----------------------------------------------------
        // State machine
        // ----------------------------------------------------

        if (
            status.fault_flags &
            (FAULT_INA226 | FAULT_CURRENT_SENSOR)
        )
        {
            status.state = PowerState::FAULT;
            status.motor_run_allowed = false;
        }

        else if (
            voltage_critical ||
            status.soc_percent <= SOC_STOP_PERCENT
        )
        {
            if (voltage_critical)
                status.fault_flags |= FAULT_LOW_VOLTAGE;

            if (status.soc_percent <= SOC_STOP_PERCENT)
                status.fault_flags |= FAULT_LOW_SOC;

            status.critical_battery = true;
            status.low_battery = true;
            status.motor_run_allowed = false;

            status.state =
                PowerState::CRITICAL_BATTERY;
        }

        else if (
            status.soc_percent <=
            SOC_WARNING_PERCENT
        )
        {
            status.low_battery = true;
            status.critical_battery = false;
            status.motor_run_allowed = true;

            status.state =
                PowerState::LOW_BATTERY;
        }

        else
        {
            status.low_battery = false;
            status.critical_battery = false;
            status.motor_run_allowed = true;

            status.state =
                PowerState::RUNNING;
        }

        status.heartbeat++;

        write_shared_status(status);

        sleep_ms(POWER_LOOP_MS);
    }
}

// ============================================================
// BUZZER
// ============================================================

static void update_buzzer(
    const PowerStatus &status)
{
    static uint64_t last_toggle_us = 0;
    static bool tone_enabled = false;

    uint64_t now = time_us_64();

    /*
     * CRITICAL_BATTERY / FAULT:
     * 250ms ON / 250ms OFF 반복
     *
     * 수동부저이므로 ON 구간에서는
     * 실제 PWM 주파수를 출력한다.
     */
    if (
        status.critical_battery ||
        status.state == PowerState::FAULT
    )
    {
        if (now - last_toggle_us >= 250000)
        {
            last_toggle_us = now;
            tone_enabled = !tone_enabled;

            if (tone_enabled)
            {
                buzzer_on(
                    BUZZER_WARN_FREQ_HZ
                );
            }
            else
            {
                buzzer_off();
            }
        }
    }
    else
    {
        tone_enabled = false;
        last_toggle_us = now;
        buzzer_off();
    }
}

// ============================================================
// ROS MESSAGE
// ============================================================

static void publish_ros_status(
    const PowerStatus &status)
{
    /*
     * ROS BatteryState에서 current는
     * 방전 시 음수.
     */

    g_battery_msg.voltage =
        status.voltage_v;

    g_battery_msg.current =
        -status.current_a;

    g_battery_msg.charge =
        status.remaining_ah;

    g_battery_msg.capacity =
        BATTERY_CAPACITY_AH;

    g_battery_msg.design_capacity =
        BATTERY_CAPACITY_AH;

    g_battery_msg.percentage =
        clampf(
            status.soc_percent / 100.0f,
            0.0f,
            1.0f
        );

    g_battery_msg.power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;

    if (
        status.state == PowerState::FAULT ||
        status.critical_battery
    )
    {
        g_battery_msg.power_supply_health =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
    }
    else
    {
        g_battery_msg.power_supply_health =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;
    }

    g_battery_msg.power_supply_technology =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;

    g_battery_msg.present = true;

    rcl_ret_t ret1 =
        rcl_publish(
            &g_battery_pub,
            &g_battery_msg,
            nullptr
        );

    (void)ret1;

    g_fault_msg.data =
        status.fault_flags;

    rcl_ret_t ret2 =
        rcl_publish(
            &g_fault_pub,
            &g_fault_msg,
            nullptr
        );

    (void)ret2;
}

// ============================================================
// SYSTEM READY ACK
// ============================================================

static bool publish_alert_ack()
{
    g_alert_ack_msg.data = true;

    rcl_ret_t ret =
        rcl_publish(
            &g_alert_ack_pub,
            &g_alert_ack_msg,
            nullptr
        );

    return ret == RCL_RET_OK;
}


// ============================================================
// CORE 0
// ============================================================

int main()
{
    // --------------------------------------------------------
    // Shared lock
    // --------------------------------------------------------

    critical_section_init(&g_status_lock);

    // --------------------------------------------------------
    // UART1 -> Motor Pico
    // --------------------------------------------------------

    uart_init(
        uart1,
        MOTOR_UART_BAUD
    );

    gpio_set_function(
        UART1_TX_PIN,
        GPIO_FUNC_UART
    );

    gpio_set_function(
        UART1_RX_PIN,
        GPIO_FUNC_UART
    );

    uart_set_format(
        uart1,
        8,
        1,
        UART_PARITY_NONE
    );

    uart_set_fifo_enabled(
        uart1,
        true
    );

    // --------------------------------------------------------
    // 가장 먼저 STOP
    // --------------------------------------------------------

    motor_stop();

    // --------------------------------------------------------
    // I2C1
    // --------------------------------------------------------

    i2c_init(
        i2c1,
        INA226_I2C_BAUD
    );

    gpio_set_function(
        VL_SDA_PIN,
        GPIO_FUNC_I2C
    );

    gpio_set_function(
        VL_SCL_PIN,
        GPIO_FUNC_I2C
    );

    gpio_pull_up(VL_SDA_PIN);
    gpio_pull_up(VL_SCL_PIN);

    gpio_init(VL_ALRT_PIN);
    gpio_set_dir(
        VL_ALRT_PIN,
        GPIO_IN
    );

    // --------------------------------------------------------
    // Buzzer PWM
    // --------------------------------------------------------

    buzzer_init();

    // --------------------------------------------------------
    // Core1
    // --------------------------------------------------------

    multicore_launch_core1(
        core1_main
    );

    while (!g_core1_ready)
    {
        // Core1 측정 완료 전까지 STOP 유지
        motor_stop();
        sleep_ms(100);
    }

    PowerStatus startup_status =
        read_shared_status();

    if (startup_status.motor_run_allowed)
    {
        /*
         * 모든 초기화/배터리 검사 통과 후에만
         * latch release.
         */
        boot_beep_ok();
        motor_run();
    }
    else
    {
        boot_beep_fault();
        motor_stop();
    }

    // --------------------------------------------------------
    // micro-ROS transport
    // --------------------------------------------------------

    /*
     * 공식 Pico micro-ROS UART transport는
     * stdio UART를 통해 통신.
     */

    rmw_uros_set_custom_transport(
        true,
        nullptr,

        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // --------------------------------------------------------
    // ROS objects
    // --------------------------------------------------------

    rcl_allocator_t allocator =
        rcl_get_default_allocator();

    /*
     * Agent가 없어도 Core1 안전제어는 계속 동작.
     *
     * 따라서 여기서 영원히 block하지 않고
     * 연결 가능한 동안만 ROS 초기화 시도.
     */

    bool ros_available = false;

    rclc_support_t support{};

    rcl_node_t node =
        rcl_get_zero_initialized_node();

    rclc_executor_t executor{};

    g_battery_pub =
        rcl_get_zero_initialized_publisher();

    g_fault_pub =
        rcl_get_zero_initialized_publisher();

    g_alert_ack_pub =
        rcl_get_zero_initialized_publisher();

    g_alert_ok_sub =
        rcl_get_zero_initialized_subscription();

    sensor_msgs__msg__BatteryState__init(
        &g_battery_msg
    );

    std_msgs__msg__UInt32__init(
        &g_fault_msg
    );

    std_msgs__msg__Bool__init(
        &g_alert_ok_msg
    );

    std_msgs__msg__Bool__init(
        &g_alert_ack_msg
    );

    // --------------------------------------------------------
    // Watchdog
    // --------------------------------------------------------

    watchdog_enable(
        2000,
        true
    );

    uint32_t previous_heartbeat =
        startup_status.heartbeat;

    uint64_t last_heartbeat_check_us =
        time_us_64();

    uint64_t last_ros_pub_us = 0;

    bool previous_motor_allowed =
        startup_status.motor_run_allowed;

    while (true)
    {
        PowerStatus status =
            read_shared_status();

        // ----------------------------------------------------
        // Motor safety transition
        // ----------------------------------------------------

        if (
            status.motor_run_allowed !=
            previous_motor_allowed
        )
        {
            if (status.motor_run_allowed)
            {
                motor_run();
            }
            else
            {
                /*
                 * Safety 명령은 여러 번 전송
                 */
                motor_stop();
                sleep_ms(10);
                motor_stop();
                sleep_ms(10);
                motor_stop();
            }

            previous_motor_allowed =
                status.motor_run_allowed;
        }

        /*
         * critical 상태에서는 일정 주기로 STOP을
         * 재전송해 모터 Pico reset 등의 상황에도 대응.
         */
        static uint64_t last_stop_resend_us = 0;

        if (!status.motor_run_allowed)
        {
            uint64_t now = time_us_64();

            if (
                now - last_stop_resend_us >=
                500000
            )
            {
                last_stop_resend_us = now;
                motor_stop();
            }
        }

        // ----------------------------------------------------
        // Buzzer
        // ----------------------------------------------------

        update_buzzer(status);

        // ----------------------------------------------------
        // System READY melody
        // ----------------------------------------------------

        if (
            g_ready_chime_request &&
            !g_ready_sequence_completed &&
            !status.critical_battery &&
            status.state != PowerState::FAULT
        )
        {
            g_ready_chime_request = false;

            // ACK is sent only AFTER the melody completed.
            // Therefore SBC knows the PCU actually processed
            // the READY event, not merely discovered the topic.
            ready_chime();

            g_ready_sequence_completed = true;
            g_alert_ack_request = true;
        }

        // ----------------------------------------------------
        // ROS agent reconnect
        // ----------------------------------------------------

        if (!ros_available)
        {
            if (
                rmw_uros_ping_agent(
                    50,
                    1
                ) == RCL_RET_OK
            )
            {
                bool init_ok = true;

                if (
                    rclc_support_init(
                        &support,
                        0,
                        nullptr,
                        &allocator
                    ) != RCL_RET_OK
                )
                {
                    init_ok = false;
                }

                if (
                    init_ok &&
                    rclc_node_init_default(
                        &node,
                        "cubic_power_pico",
                        "",
                        &support
                    ) != RCL_RET_OK
                )
                {
                    init_ok = false;
                }

                if (
                    init_ok &&
                    rclc_publisher_init_default(
                        &g_battery_pub,
                        &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(
                            sensor_msgs,
                            msg,
                            BatteryState
                        ),
                        "/power/battery"
                    ) != RCL_RET_OK
                )
                {
                    init_ok = false;
                }

                if (
                    init_ok &&
                    rclc_publisher_init_default(
                        &g_fault_pub,
                        &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(
                            std_msgs,
                            msg,
                            UInt32
                        ),
                        "/power/fault"
                    ) != RCL_RET_OK
                )
                {
                    init_ok = false;
                }

                if (
                    init_ok &&
                    rclc_publisher_init_default(
                        &g_alert_ack_pub,
                        &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(
                            std_msgs,
                            msg,
                            Bool
                        ),
                        "/system/alert_ack"
                    ) != RCL_RET_OK
                )
                {
                    init_ok = false;
                }

                if (
                    init_ok &&
                    rclc_subscription_init_default(
                        &g_alert_ok_sub,
                        &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(
                            std_msgs,
                            msg,
                            Bool
                        ),
                        "/system/alert_ok"
                    ) != RCL_RET_OK
                )
                {
                    init_ok = false;
                }

                if (init_ok)
                {
                    if (
                        rclc_executor_init(
                            &executor,
                            &support.context,
                            1,
                            &allocator
                        ) != RCL_RET_OK
                    )
                    {
                        init_ok = false;
                    }
                }

                if (init_ok)
                {
                    if (
                        rclc_executor_add_subscription(
                            &executor,
                            &g_alert_ok_sub,
                            &g_alert_ok_msg,
                            &alert_ok_callback,
                            ON_NEW_DATA
                        ) != RCL_RET_OK
                    )
                    {
                        init_ok = false;
                    }
                }

                if (init_ok)
                {
                    ros_available = true;
                }
            }
        }

        // ----------------------------------------------------
        // ROS publish
        // ----------------------------------------------------

        if (ros_available)
        {
            uint64_t now =
                time_us_64();

            if (
                now - last_ros_pub_us >=
                ROS_PUBLISH_MS * 1000ULL
            )
            {
                last_ros_pub_us = now;

                publish_ros_status(status);
            }

            rclc_executor_spin_some(
                &executor,
                RCL_MS_TO_NS(1)
            );

            // If ACK delivery itself races DDS discovery, the
            // SBC keeps retransmitting alert_ok. The callback
            // above then requests another ACK without replaying
            // the melody.
            if (g_alert_ack_request)
            {
                if (publish_alert_ack())
                {
                    g_alert_ack_request = false;
                }
            }
        }

        // ----------------------------------------------------
        // Core1 watchdog heartbeat
        // ----------------------------------------------------

        uint64_t now_us =
            time_us_64();

        if (
            now_us - last_heartbeat_check_us >=
            500000
        )
        {
            last_heartbeat_check_us = now_us;

            if (
                status.heartbeat !=
                previous_heartbeat
            )
            {
                previous_heartbeat =
                    status.heartbeat;

                /*
                 * Core0 살아있고
                 * Core1도 살아있는 경우만 watchdog feed.
                 */
                watchdog_update();
            }

            /*
             * heartbeat가 안 변하면 watchdog_update를
             * 하지 않아 약 2초 후 RP2350 reset.
             *
             * reset 후 main() 최초 동작 = Motor STOP.
             */
        }

        sleep_ms(1);
    }

    return 0;
}