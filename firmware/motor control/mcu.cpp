#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

#include "hardware/uart.h"

extern "C"
{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rmw_microros/rmw_microros.h>

#include "pico_uart_transports.h"
}


// ============================================================
// CUBIC C1
// RP2350 Motor Controller
//
// CORE 0
//  - micro-ROS : /cmd_vel subscriber
//  - UART1     : Power Pico
//  - ADC0      : M0 FG
//  - ADC1      : M1 FG
//  - DMA       : ADC sampling
//  - FG hysteresis
//  - FG speed calculation
//  - speed filtering
//  - cmd_vel -> motor FG ticks/s
//  - software command watchdog
//
// CORE 1
//  - acceleration limiting
//  - one-shot startup boost
//  - P control
//  - coast deceleration
//  - direction change protection
//  - PWM / DIR
//
// IMPORTANT CONTROL PHILOSOPHY
//
// FG provides speed information, but NOT direction information.
//
// Therefore:
//  1. Normal acceleration:
//       PWM is applied in the commanded direction.
//  2. Target speed is already reached/exceeded:
//       PWM = 0 -> motor coasts naturally.
//  3. STOP:
//       PWM = 0 -> motor coasts naturally.
//  4. Direction change:
//       PWM = 0 -> wait until motor nearly stops
//       -> change DIR
//       -> startup boost.
//  5. Startup:
//       Startup boost is allowed only once after STOP,
//       or once after a safe direction change.
//
// The controller NEVER applies reverse PWM merely to brake.
//
// PIO : NOT USED
// ============================================================


// ============================================================
// PIN CONFIGURATION
// ============================================================

// Motor 0 / Left
constexpr uint M0_FG_PIN  = 26;
constexpr uint M0_DIR_PIN = 21;
constexpr uint M0_PWM_PIN = 19;

// Motor 1 / Right
constexpr uint M1_FG_PIN  = 27;
constexpr uint M1_DIR_PIN = 20;
constexpr uint M1_PWM_PIN = 18;

// Upper Controller UART0
constexpr uint UART0_TX_PIN = 0;
constexpr uint UART0_RX_PIN = 1;

// Power Pico UART1
constexpr uint UART1_TX_PIN = 4;
constexpr uint UART1_RX_PIN = 5;


// ============================================================
// UART
// ============================================================

constexpr uint UART_BAUD = 115200;

uart_inst_t* UPPER_UART = uart0;
uart_inst_t* POWER_UART = uart1;


// ============================================================
// MOTOR / GEAR / WHEEL
// ============================================================

constexpr float FG_PPR = 9.0f;

constexpr float GEAR_RATIO = 26.67f;

constexpr float MOTOR_MAX_RPM = 8000.0f;

constexpr float MAX_TICKS_PER_SEC =
    MOTOR_MAX_RPM * FG_PPR / 60.0f;

constexpr float WHEEL_DIAMETER_M = 0.151f;

constexpr float WHEEL_CIRCUMFERENCE_M =
    static_cast<float>(M_PI) *
    WHEEL_DIAMETER_M;

constexpr float WHEEL_TRACK_M = 0.395f;

// ============================================================
// MINIMUM PRACTICAL MOTOR SPEED
// ============================================================
//
// Actual vehicle test:
//
// Below approximately 40% PWM the motor cannot reliably
// maintain rotation under vehicle load.
//
// The practical minimum vehicle speed is about 0.12 m/s,
// corresponding to roughly 60 FG ticks/s.
//
// Any non-zero target below this value is raised to this
// minimum.
//
// Zero remains zero.
// ============================================================

constexpr float MIN_MOTOR_TICKS_PER_SEC = 60.0f;


// ============================================================
// ACCELERATION LIMIT
// ============================================================

constexpr float MAX_ACCEL_TICKS_PER_SEC2 = 2500.0f;


// ============================================================
// P CONTROL / DEAD ZONE
// ============================================================
//
// Runtime tunable.
//
// Normal driving:
//
//   error <= 0
//       PWM = 0 -> COAST
//
//   error > 0
//       PWM = PWM_MIN_RUN + Kp * error
//
// The motor experimentally starts responding at approximately
// 40% PWM, therefore PWM below this region is not useful for
// normal closed-loop driving.
// ============================================================

volatile float control_kp = 0.80f;

// 0 ~ 1000 = 0 ~ 100%
volatile float pwm_min_run = 400.0f;

constexpr float KP_MIN = 0.0f;
constexpr float KP_MAX = 20.0f;

constexpr float PWM_MIN_RUN_MIN = 400.0f;
constexpr float PWM_MIN_RUN_MAX = 400.0f;

constexpr float SPEED_DB_ENTER = 30.0f;   // 목표보다 30 이상 빠르면 coast
constexpr float SPEED_DB_EXIT  = 30.0f;   // 목표보다 30 이상 느리면 drive


// ============================================================
// PWM SLEW RATE
// ============================================================
//
// PWM scale:
//   0 ~ 1000 = 0 ~ 100%
//
// Normal control does NOT jump directly between PWM values.
//
// Acceleration:
//   relatively fast
//
// Deceleration:
//   slower / smoother
//
// STOP is the only exception:
//   PWM immediately becomes 0.
//
// 100 Hz control:
//
// PWM_RISE_RATE = 1000 / sec
//   -> +10 per cycle = +1.0% duty / 10 ms
//
// PWM_FALL_RATE = 250 / sec
//   -> -2.5 per cycle = -0.25% duty / 10 ms
// ============================================================

constexpr float PWM_RISE_RATE = 1000.0f;
constexpr float PWM_FALL_RATE = 250.0f;

constexpr float PWM_ZERO_THRESHOLD = 1.0f;

constexpr float PWM_FREQUENCY = 20000.0f;

constexpr uint16_t PWM_WRAP = 1000;

constexpr float PWM_MAX_COMMAND = 650.0f;


// ============================================================
// MOTOR STARTUP
// ============================================================
//
// Startup boost is ONE-SHOT.
//
// It is armed while the motor is commanded STOP.
// When motion begins:
//
//     armed -> startup -> normal control
//
// After startup exits, low actual speed does NOT retrigger it.
//
// Direction changes explicitly start a new startup cycle after
// the motor has coasted below DIRECTION_CHANGE_SPEED.
// ============================================================

constexpr float PWM_START_DUTY = 400.0f;

constexpr float STARTUP_MAX_EXIT_SPEED = 80.0f;

constexpr float STARTUP_EXIT_FRACTION = 0.80f;

constexpr float STARTUP_ENTRY_SPEED = 5.0f;

constexpr uint32_t STARTUP_TIMEOUT_MS = 500;


// ============================================================
// DIRECTION CHANGE
// ============================================================

constexpr float DIRECTION_CHANGE_SPEED = 50.0f;


// ============================================================
// CONTROL LOOP
// ============================================================

constexpr uint32_t CONTROL_PERIOD_US = 10000;

// FG speed estimation window: 50 ms
constexpr uint32_t FG_SPEED_PERIOD_US = 50000;

// ============================================================
// SPEED FILTER
// ============================================================

constexpr float SPEED_FILTER_ALPHA = 0.30f;


// ============================================================
// FG ADC HYSTERESIS
// ============================================================

constexpr uint16_t FG_HIGH_THRESHOLD = 3537;
constexpr uint16_t FG_LOW_THRESHOLD  = 2792;


// ============================================================
// ADC / DMA
// ============================================================

constexpr float ADC_TOTAL_SAMPLE_RATE = 50000.0f;

// RP2350 ADC clock divider for approximately 50 kS/s total.
constexpr float ADC_CLKDIV = 959.0f;

constexpr uint ADC_DMA_BUFFER_SIZE = 256;

// Ping-pong DMA buffers.
// While one DMA channel is filling one buffer, the other completed
// buffer is processed and re-armed from DMA IRQ 0.
alignas(4) uint16_t adc_dma_buffer_a[
    ADC_DMA_BUFFER_SIZE
];

alignas(4) uint16_t adc_dma_buffer_b[
    ADC_DMA_BUFFER_SIZE
];

int adc_dma_channel_a = -1;
int adc_dma_channel_b = -1;

// Forward declarations used by the DMA setup/IRQ path.
void process_adc_buffer(
    const uint16_t* buffer
);

void adc_dma_irq_handler();


// ============================================================
// FG DIGITAL FILTER
// ============================================================
//
// ADC total sampling:
//   50 kS/s total
//
// Two channels round-robin:
//   approximately 25 kS/s per motor
//
// Per-channel sample interval:
//   approximately 40 us
//
// Motor maximum FG:
//   1200 Hz
//
// Minimum physical FG period:
//   approximately 833 us
//
// We reject repeated edges that occur too quickly.
//
// FILTER:
//   1. ADC hysteresis
//   2. consecutive sample confirmation
//   3. minimum edge interval
// ============================================================

constexpr uint8_t FG_CONFIRM_SAMPLES = 3;

// 12 samples × ~40 us = ~480 us
constexpr uint16_t FG_MIN_EDGE_SAMPLES = 12;


// ============================================================
// FG STATE
// ============================================================

struct FGState
{
    bool high = false;

    volatile uint32_t pulse_count = 0;

    uint32_t previous_pulse_count = 0;

    uint8_t high_confirm_count = 0;
    uint8_t low_confirm_count = 0;

    uint16_t samples_since_edge = 0xFFFF;
};

FGState fg_m0;
FGState fg_m1;


// ============================================================
// SHARED SPEED DATA
// ============================================================

volatile float m0_actual_raw_ticks_s = 0.0f;
volatile float m1_actual_raw_ticks_s = 0.0f;

volatile float m0_actual_ticks_s = 0.0f;
volatile float m1_actual_ticks_s = 0.0f;


// ============================================================
// SHARED TARGET DATA
// ============================================================

volatile float m0_target_ticks_s = 0.0f;
volatile float m1_target_ticks_s = 0.0f;


// ============================================================
// micro-ROS / CMD_VEL WATCHDOG
// ============================================================
//
// /cmd_vel:
//   linear.x  -> vehicle linear velocity [m/s]
//   angular.z -> vehicle yaw rate [rad/s]
//
// A Twist with linear.x == 0 and angular.z == 0 produces
// zero motor targets.
//
// Software watchdog:
//   If no fresh /cmd_vel is received for CMD_VEL_TIMEOUT_MS,
//   Core 0 forces both motor targets to zero.
//
// NOTE:
//   This watchdog requests a normal target STOP and therefore
//   still follows the Core 1 acceleration/control path.
//   UART1 Power Pico STOP is separate and performs an immediate
//   hard-stop that bypasses the normal control path.
// ============================================================

constexpr uint32_t CMD_VEL_TIMEOUT_MS = 500;
constexpr uint64_t CMD_VEL_TIMEOUT_US =
    static_cast<uint64_t>(CMD_VEL_TIMEOUT_MS) * 1000ULL;

rcl_subscription_t cmd_vel_subscription;
geometry_msgs__msg__Twist cmd_vel_msg;

// Wheel odometry publisher.
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

// ============================================================
// micro-ROS RECONNECTION STATE
// ============================================================
//
// The MCU must not depend on SBC/Agent boot order.
//
// WAITING_AGENT
//   -> periodically ping until Agent appears.
//
// AGENT_AVAILABLE
//   -> create ROS entities.
//
// AGENT_CONNECTED
//   -> spin executor + publish odom.
//   -> periodically health-check Agent.
//
// AGENT_DISCONNECTED
//   -> force command STOP, destroy stale entities,
//      then return to WAITING_AGENT.
//
// This allows:
//   - MCU boot before SBC
//   - SBC reboot while MCU remains powered
//   - micro-ROS Agent restart
// without requiring a Pico reset.
// ============================================================

enum class MicroRosState : uint8_t
{
    WAITING_AGENT = 0,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

constexpr uint32_t ROS_WAIT_PING_INTERVAL_MS = 500;
constexpr uint32_t ROS_CONNECTED_PING_INTERVAL_MS = 1000;
constexpr uint32_t ROS_PING_TIMEOUT_MS = 100;
constexpr uint8_t ROS_DISCONNECT_MISSES = 3;

MicroRosState micro_ros_state =
    MicroRosState::WAITING_AGENT;

uint64_t last_ros_agent_check_us = 0;
uint8_t ros_agent_miss_count = 0;

// ROS objects must persist across reconnect cycles.
rcl_allocator_t ros_allocator;
rclc_support_t ros_support{};
rcl_node_t ros_node =
    rcl_get_zero_initialized_node();

rclc_executor_t ros_executor =
    rclc_executor_get_zero_initialized_executor();

bool ros_support_initialized = false;
bool ros_node_initialized = false;
bool ros_subscription_initialized = false;
bool ros_publisher_initialized = false;
bool ros_executor_initialized = false;
bool odom_message_initialized = false;

// 20 Hz odometry update/publish rate.
constexpr uint32_t ODOM_PERIOD_US = 50000;

// Integrated planar wheel odometry state.
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_yaw = 0.0f;

// Reset only the integration time base on ROS reconnect.
// Keep accumulated x/y/yaw.
uint64_t odom_previous_update_us = 0;

volatile uint64_t last_cmd_vel_us = 0;
volatile bool cmd_vel_received = false;
volatile bool cmd_vel_watchdog_active = true;


// ============================================================
// POWER PICO HARD-STOP
// ============================================================
//
// UART1 commands:
//   STOP -> latch immediate hard-stop
//   RUN  -> release hard-stop latch
//
// While hard-stop is active:
//   - Core 0 forces motor targets to zero.
//   - /cmd_vel cannot restart the motors.
//   - Core 1 immediately forces both PWM outputs to zero,
//     bypassing acceleration limiting and PWM slew.
//
// RUN only releases the latch. A fresh /cmd_vel is still
// required before motion resumes.
// ============================================================

volatile bool power_hard_stop_active = false;

// Shared Core0 -> Core1 hard-stop state.
volatile bool core1_hard_stop_request = false;


// ============================================================
// SHARED CONTROL DEBUG DATA
// ============================================================

volatile float m0_control_error = 0.0f;
volatile float m1_control_error = 0.0f;

volatile float m0_pwm_command = 0.0f;
volatile float m1_pwm_command = 0.0f;

volatile float m0_limited_target = 0.0f;
volatile float m1_limited_target = 0.0f;

volatile bool m0_startup_active = false;
volatile bool m1_startup_active = false;

volatile bool m0_startup_armed = true;
volatile bool m1_startup_armed = true;

volatile bool m0_coasting = false;
volatile bool m1_coasting = false;

volatile bool m0_direction_change_wait = false;
volatile bool m1_direction_change_wait = false;

volatile int m0_direction_state = 1;
volatile int m1_direction_state = 1;


// ============================================================
// HELPER
// ============================================================

float clamp_float(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum)
        return minimum;

    if (value > maximum)
        return maximum;

    return value;
}

float slew_pwm(
    float current,
    float target,
    float dt
)
{
    target =
        clamp_float(
            target,
            0.0f,
            PWM_MAX_COMMAND
        );

    float max_rise =
        PWM_RISE_RATE *
        dt;

    float max_fall =
        PWM_FALL_RATE *
        dt;


    // Rise
    if (target > current)
    {
        current +=
            max_rise;

        if (current > target)
        {
            current =
                target;
        }
    }

    // Fall
    else if (target < current)
    {
        current -=
            max_fall;

        if (current < target)
        {
            current =
                target;
        }
    }


    if (
        current <
        PWM_ZERO_THRESHOLD
    )
    {
        current = 0.0f;
    }


    return current;
}

// ============================================================
// PWM SETUP
// ============================================================

void setup_pwm(
    uint pwm_pin
)
{
    gpio_set_function(
        pwm_pin,
        GPIO_FUNC_PWM
    );

    uint slice =
        pwm_gpio_to_slice_num(
            pwm_pin
        );

    pwm_config config =
        pwm_get_default_config();

    float divider =
        125000000.0f /
        (
            PWM_FREQUENCY *
            (PWM_WRAP + 1)
        );

    pwm_config_set_clkdiv(
        &config,
        divider
    );

    pwm_config_set_wrap(
        &config,
        PWM_WRAP
    );

    pwm_init(
        slice,
        &config,
        true
    );

    pwm_set_gpio_level(
        pwm_pin,
        0
    );
}


// ============================================================
// DIR SETUP
// ============================================================

void setup_direction_pin(
    uint dir_pin
)
{
    gpio_init(dir_pin);

    gpio_set_dir(
        dir_pin,
        GPIO_OUT
    );

    gpio_put(
        dir_pin,
        0
    );
}


// ============================================================
// MOTOR OUTPUT
// ============================================================

void set_motor_output(
    uint pwm_pin,
    uint dir_pin,
    float command,
    bool direction_inverted
)
{
    command =
        clamp_float(
            command,
            -PWM_MAX_COMMAND,
            PWM_MAX_COMMAND
        );

    if (fabsf(command) < 0.001f)
    {
        pwm_set_gpio_level(
            pwm_pin,
            0
        );

        return;
    }

    bool forward =
        command > 0.0f;

    if (direction_inverted)
    {
        forward = !forward;
    }

    gpio_put(
        dir_pin,
        forward ? 1 : 0
    );

    float duty =
        fabsf(command);

    duty =
        clamp_float(
            duty,
            0.0f,
            PWM_MAX_COMMAND
        );

    pwm_set_gpio_level(
        pwm_pin,
        static_cast<uint16_t>(
            duty
        )
    );
}


// ============================================================
// ADC SETUP
// ============================================================

void setup_adc()
{
    adc_init();

    adc_gpio_init(
        M0_FG_PIN
    );

    adc_gpio_init(
        M1_FG_PIN
    );

    gpio_pull_up(
        M0_FG_PIN
    );

    gpio_pull_up(
        M1_FG_PIN
    );

    adc_select_input(0);

    adc_set_round_robin(
        (1u << 0) |
        (1u << 1)
    );

    adc_fifo_setup(
        true,
        true,
        1,
        false,
        false
    );

    adc_set_clkdiv(
        ADC_CLKDIV
    );
}


// ============================================================
// ADC PING-PONG DMA SETUP
// ============================================================
//
// Channel A -> buffer A -> chain to B
// Channel B -> buffer B -> chain to A
//
// DMA completion raises IRQ 0. The IRQ handler:
//   1. clears the completed-channel IRQ,
//   2. processes that completed buffer,
//   3. restores its write address and transfer count.
//
// Because the opposite DMA channel has already started through
// hardware chaining, ADC acquisition continues while the CPU
// processes/re-arms the completed buffer.
// ============================================================

void setup_adc_dma()
{
    adc_dma_channel_a =
        dma_claim_unused_channel(
            true
        );

    adc_dma_channel_b =
        dma_claim_unused_channel(
            true
        );

    dma_channel_config config_a =
        dma_channel_get_default_config(
            adc_dma_channel_a
        );

    channel_config_set_transfer_data_size(
        &config_a,
        DMA_SIZE_16
    );

    channel_config_set_read_increment(
        &config_a,
        false
    );

    channel_config_set_write_increment(
        &config_a,
        true
    );

    channel_config_set_dreq(
        &config_a,
        DREQ_ADC
    );

    channel_config_set_chain_to(
        &config_a,
        adc_dma_channel_b
    );

    dma_channel_configure(
        adc_dma_channel_a,
        &config_a,
        adc_dma_buffer_a,
        &adc_hw->fifo,
        ADC_DMA_BUFFER_SIZE,
        false
    );

    dma_channel_config config_b =
        dma_channel_get_default_config(
            adc_dma_channel_b
        );

    channel_config_set_transfer_data_size(
        &config_b,
        DMA_SIZE_16
    );

    channel_config_set_read_increment(
        &config_b,
        false
    );

    channel_config_set_write_increment(
        &config_b,
        true
    );

    channel_config_set_dreq(
        &config_b,
        DREQ_ADC
    );

    channel_config_set_chain_to(
        &config_b,
        adc_dma_channel_a
    );

    dma_channel_configure(
        adc_dma_channel_b,
        &config_b,
        adc_dma_buffer_b,
        &adc_hw->fifo,
        ADC_DMA_BUFFER_SIZE,
        false
    );

    dma_channel_set_irq0_enabled(
        adc_dma_channel_a,
        true
    );

    dma_channel_set_irq0_enabled(
        adc_dma_channel_b,
        true
    );

    irq_set_exclusive_handler(
        DMA_IRQ_0,
        adc_dma_irq_handler
    );

    irq_set_enabled(
        DMA_IRQ_0,
        true
    );

    adc_run(false);
    adc_fifo_drain();

    dma_channel_start(
        adc_dma_channel_a
    );

    adc_run(true);
}


// ============================================================
// FG HYSTERESIS
// ============================================================

void process_fg_sample(
    uint16_t adc_value,
    FGState& state
)
{
    // --------------------------------------------------------
    // Samples elapsed since last accepted edge
    // --------------------------------------------------------

    if (
        state.samples_since_edge <
        0xFFFF
    )
    {
        state.samples_since_edge++;
    }


    // ========================================================
    // LOW STATE
    // ========================================================

    if (!state.high)
    {
        // ----------------------------------------------------
        // During minimum edge interval, ignore HIGH spikes.
        // ----------------------------------------------------

        if (
            state.samples_since_edge <
            FG_MIN_EDGE_SAMPLES
        )
        {
            state.high_confirm_count = 0;

            return;
        }


        // ----------------------------------------------------
        // HIGH threshold confirmation
        // ----------------------------------------------------

        if (
            adc_value >
            FG_HIGH_THRESHOLD
        )
        {
            if (
                state.high_confirm_count <
                0xFF
            )
            {
                state.high_confirm_count++;
            }


            if (
                state.high_confirm_count >=
                FG_CONFIRM_SAMPLES
            )
            {
                state.high = true;

                state.pulse_count++;

                state.samples_since_edge = 0;

                state.high_confirm_count = 0;
                state.low_confirm_count = 0;
            }
        }
        else
        {
            state.high_confirm_count = 0;
        }
    }


    // ========================================================
    // HIGH STATE
    // ========================================================

    else
    {
        // ----------------------------------------------------
        // LOW threshold confirmation
        // ----------------------------------------------------

        if (
            adc_value <
            FG_LOW_THRESHOLD
        )
        {
            if (
                state.low_confirm_count <
                0xFF
            )
            {
                state.low_confirm_count++;
            }


            if (
                state.low_confirm_count >=
                FG_CONFIRM_SAMPLES
            )
            {
                state.high = false;

                state.low_confirm_count = 0;
                state.high_confirm_count = 0;
            }
        }
        else
        {
            state.low_confirm_count = 0;
        }
    }
}

// ============================================================
// PROCESS COMPLETED DMA BLOCK
// ============================================================

void process_adc_buffer(
    const uint16_t* buffer
)
{
    for (
        uint i = 0;
        i < ADC_DMA_BUFFER_SIZE;
        i++
    )
    {
        uint16_t value =
            buffer[i] &
            0x0FFF;

        if ((i & 1u) == 0)
        {
            process_fg_sample(
                value,
                fg_m0
            );
        }
        else
        {
            process_fg_sample(
                value,
                fg_m1
            );
        }
    }
}


// ============================================================
// DMA IRQ 0
// ============================================================
//
// The opposite DMA channel is already running when this handler
// executes because each channel is hardware-chained to the other.
//
// The completed channel must be re-armed before the active channel
// finishes its next block. With 256 samples at 50 kS/s total, the
// available re-arm window is about 5.12 ms.
// ============================================================

void adc_dma_irq_handler()
{
    uint32_t pending =
        dma_hw->ints0;

    if (
        pending &
        (1u << adc_dma_channel_a)
    )
    {
        dma_hw->ints0 =
            (1u << adc_dma_channel_a);

        process_adc_buffer(
            adc_dma_buffer_a
        );

        dma_channel_set_write_addr(
            adc_dma_channel_a,
            adc_dma_buffer_a,
            false
        );

        dma_channel_set_trans_count(
            adc_dma_channel_a,
            ADC_DMA_BUFFER_SIZE,
            false
        );
    }

    if (
        pending &
        (1u << adc_dma_channel_b)
    )
    {
        dma_hw->ints0 =
            (1u << adc_dma_channel_b);

        process_adc_buffer(
            adc_dma_buffer_b
        );

        dma_channel_set_write_addr(
            adc_dma_channel_b,
            adc_dma_buffer_b,
            false
        );

        dma_channel_set_trans_count(
            adc_dma_channel_b,
            ADC_DMA_BUFFER_SIZE,
            false
        );
    }
}


// ============================================================
// FG SPEED UPDATE
// ============================================================

void update_fg_speed()
{
    static uint64_t previous_time_us = 0;

    uint64_t now =
        time_us_64();

    if (previous_time_us == 0)
    {
        previous_time_us =
            now;

        fg_m0.previous_pulse_count =
            fg_m0.pulse_count;

        fg_m1.previous_pulse_count =
            fg_m1.pulse_count;

        return;
    }

    uint64_t elapsed_us =
        now -
        previous_time_us;

    if (
        elapsed_us <
        FG_SPEED_PERIOD_US
    )
    {
        return;
    }

    float dt =
        elapsed_us /
        1000000.0f;

    uint32_t m0_delta =
        fg_m0.pulse_count -
        fg_m0.previous_pulse_count;

    uint32_t m1_delta =
        fg_m1.pulse_count -
        fg_m1.previous_pulse_count;

    fg_m0.previous_pulse_count =
        fg_m0.pulse_count;

    fg_m1.previous_pulse_count =
        fg_m1.pulse_count;

    float raw_m0 =
        static_cast<float>(
            m0_delta
        ) / dt;

    float raw_m1 =
        static_cast<float>(
            m1_delta
        ) / dt;

    m0_actual_raw_ticks_s =
        raw_m0;

    m1_actual_raw_ticks_s =
        raw_m1;

    m0_actual_ticks_s +=
        SPEED_FILTER_ALPHA *
        (
            raw_m0 -
            m0_actual_ticks_s
        );

    m1_actual_ticks_s +=
        SPEED_FILTER_ALPHA *
        (
            raw_m1 -
            m1_actual_ticks_s
        );

    previous_time_us =
        now;
}


// ============================================================
// CMD_VEL -> MOTOR FG TICKS/S
// ============================================================

void cmd_vel_to_ticks(
    float v,
    float omega,
    volatile float& m0_ticks,
    volatile float& m1_ticks
)
{
    float left_velocity =
        v -
        (
            omega *
            WHEEL_TRACK_M /
            2.0f
        );

    float right_velocity =
        v +
        (
            omega *
            WHEEL_TRACK_M /
            2.0f
        );

    float left_rps =
        left_velocity /
        WHEEL_CIRCUMFERENCE_M;

    float right_rps =
        right_velocity /
        WHEEL_CIRCUMFERENCE_M;

    float left_rpm =
        left_rps *
        60.0f;

    float right_rpm =
        right_rps *
        60.0f;

    float left_motor_rpm =
        left_rpm *
        GEAR_RATIO;

    float right_motor_rpm =
        right_rpm *
        GEAR_RATIO;

    // Motor RPM -> FG ticks/s.

    m0_ticks =
        left_motor_rpm *
        FG_PPR /
        60.0f;

    m1_ticks =
        right_motor_rpm *
        FG_PPR /
        60.0f;


    // --------------------------------------------------------
    // Practical minimum motor speed
    // --------------------------------------------------------

    if (
        fabsf(m0_ticks) > 0.001f &&
        fabsf(m0_ticks) <
        MIN_MOTOR_TICKS_PER_SEC
    )
    {
        m0_ticks =
            m0_ticks > 0.0f
            ? MIN_MOTOR_TICKS_PER_SEC
            : -MIN_MOTOR_TICKS_PER_SEC;
    }

    if (
        fabsf(m1_ticks) > 0.001f &&
        fabsf(m1_ticks) <
        MIN_MOTOR_TICKS_PER_SEC
    )
    {
        m1_ticks =
            m1_ticks > 0.0f
            ? MIN_MOTOR_TICKS_PER_SEC
            : -MIN_MOTOR_TICKS_PER_SEC;
    }


    // --------------------------------------------------------
    // Maximum safety clamp
    // --------------------------------------------------------

    m0_ticks =
        clamp_float(
            m0_ticks,
            -MAX_TICKS_PER_SEC,
            MAX_TICKS_PER_SEC
        );

    m1_ticks =
        clamp_float(
            m1_ticks,
            -MAX_TICKS_PER_SEC,
            MAX_TICKS_PER_SEC
        );
}



// ============================================================
// micro-ROS CMD_VEL CALLBACK
// ============================================================

void cmd_vel_callback(
    const void* msgin
)
{
    const geometry_msgs__msg__Twist* msg =
        static_cast<const geometry_msgs__msg__Twist*>(
            msgin
        );

    const float v =
        static_cast<float>(
            msg->linear.x
        );

    const float omega =
        static_cast<float>(
            msg->angular.z
        );


    // --------------------------------------------------------
    // POWER PICO HARD-STOP OVERRIDE
    // --------------------------------------------------------

    if (power_hard_stop_active)
    {
        m0_target_ticks_s = 0.0f;
        m1_target_ticks_s = 0.0f;

        last_cmd_vel_us = time_us_64();
        cmd_vel_received = true;
        cmd_vel_watchdog_active = false;

        return;
    }


    // --------------------------------------------------------
    // ROS 2 has no separate STOP command here.
    //
    // cmd_vel 0 0 -> target 0 0.
    // --------------------------------------------------------

    if (
        fabsf(v) < 0.000001f &&
        fabsf(omega) < 0.000001f
    )
    {
        m0_target_ticks_s = 0.0f;
        m1_target_ticks_s = 0.0f;
    }
    else
    {
        cmd_vel_to_ticks(
            v,
            omega,
            m0_target_ticks_s,
            m1_target_ticks_s
        );
    }


    last_cmd_vel_us =
        time_us_64();

    cmd_vel_received = true;
    cmd_vel_watchdog_active = false;
}


// ============================================================
// SOFTWARE COMMAND WATCHDOG
// ============================================================

void update_cmd_vel_watchdog(
    uint64_t now_us
)
{
    if (!cmd_vel_received)
    {
        // Boot state is STOP until the first valid cmd_vel.
        m0_target_ticks_s = 0.0f;
        m1_target_ticks_s = 0.0f;

        cmd_vel_watchdog_active = true;

        return;
    }


    if (
        now_us -
        last_cmd_vel_us
        >= CMD_VEL_TIMEOUT_US
    )
    {
        m0_target_ticks_s = 0.0f;
        m1_target_ticks_s = 0.0f;

        cmd_vel_watchdog_active = true;
    }
}


// ============================================================
// WHEEL ODOMETRY
// ============================================================
//
// FG speed is a magnitude. Logical motor direction is restored
// with m0_direction_state / m1_direction_state.
//
// left/right wheel speed [m/s]:
//   ticks/s / FG_PPR / GEAR_RATIO * wheel circumference
//
// Differential drive:
//   v     = (v_left + v_right) / 2
//   omega = (v_right - v_left) / wheel_track
//
// Pose is integrated in the odom frame and published as
// nav_msgs/msg/Odometry on /odom.
// ============================================================

void update_and_publish_odometry(
    uint64_t now_us
)
{
    if (odom_previous_update_us == 0)
    {
        odom_previous_update_us = now_us;
        return;
    }

    uint64_t elapsed_us =
        now_us - odom_previous_update_us;

    if (elapsed_us < ODOM_PERIOD_US)
    {
        return;
    }

    float dt =
        static_cast<float>(elapsed_us) /
        1000000.0f;

    // Restore logical wheel direction. FG itself has no sign.
    float left_ticks_s =
        m0_actual_ticks_s *
        static_cast<float>(m0_direction_state);

    float right_ticks_s =
        m1_actual_ticks_s *
        static_cast<float>(m1_direction_state);

    float left_velocity =
        left_ticks_s /
        FG_PPR /
        GEAR_RATIO *
        WHEEL_CIRCUMFERENCE_M;

    float right_velocity =
        right_ticks_s /
        FG_PPR /
        GEAR_RATIO *
        WHEEL_CIRCUMFERENCE_M;

    float linear_velocity =
        0.5f *
        (left_velocity + right_velocity);

    float angular_velocity =
        (right_velocity - left_velocity) /
        WHEEL_TRACK_M;

    // Midpoint integration gives better curved-motion behavior
    // than integrating x/y using only the old yaw.
    float delta_yaw =
        angular_velocity * dt;

    float yaw_mid =
        odom_yaw +
        0.5f * delta_yaw;

    odom_x +=
        linear_velocity *
        cosf(yaw_mid) *
        dt;

    odom_y +=
        linear_velocity *
        sinf(yaw_mid) *
        dt;

    odom_yaw +=
        delta_yaw;

    // Keep yaw bounded to avoid unbounded float growth.
    while (odom_yaw > static_cast<float>(M_PI))
    {
        odom_yaw -=
            2.0f * static_cast<float>(M_PI);
    }

    while (odom_yaw < -static_cast<float>(M_PI))
    {
        odom_yaw +=
            2.0f * static_cast<float>(M_PI);
    }

    // ROS time synchronized to the micro-ROS Agent.
    int64_t epoch_ns =
        rmw_uros_epoch_nanos();

    if (epoch_ns > 0)
    {
        odom_msg.header.stamp.sec =
            static_cast<int32_t>(
                epoch_ns / 1000000000LL
            );

        odom_msg.header.stamp.nanosec =
            static_cast<uint32_t>(
                epoch_ns % 1000000000LL
            );
    }
    else
    {
        // Safe fallback if time synchronization is unavailable.
        odom_msg.header.stamp.sec =
            static_cast<int32_t>(
                now_us / 1000000ULL
            );

        odom_msg.header.stamp.nanosec =
            static_cast<uint32_t>(
                (now_us % 1000000ULL) * 1000ULL
            );
    }

    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;

    // Planar yaw -> quaternion.
    float half_yaw =
        0.5f * odom_yaw;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sinf(half_yaw);
    odom_msg.pose.pose.orientation.w = cosf(half_yaw);

    odom_msg.twist.twist.linear.x =
        linear_velocity;

    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z =
        angular_velocity;

    // Pose covariance
    odom_msg.pose.covariance[0]  = 0.02;   // x
    odom_msg.pose.covariance[7]  = 0.02;   // y
    odom_msg.pose.covariance[14] = 1e6;    // z
    odom_msg.pose.covariance[21] = 1e6;    // roll
    odom_msg.pose.covariance[28] = 1e6;    // pitch
    odom_msg.pose.covariance[35] = 0.10;   // yaw

    // Twist covariance
    odom_msg.twist.covariance[0]  = 0.02;  // vx
    odom_msg.twist.covariance[7]  = 0.05;  // vy
    odom_msg.twist.covariance[14] = 1e6;   // vz
    odom_msg.twist.covariance[21] = 1e6;   // vroll
    odom_msg.twist.covariance[28] = 1e6;   // vpitch
    odom_msg.twist.covariance[35] = 0.05;  // vyaw

    // Publishing failure must never block motor control.
    (void)rcl_publish(
        &odom_publisher,
        &odom_msg,
        NULL
    );

    odom_previous_update_us =
        now_us;
}


// ============================================================
// COMMAND PROCESSOR
// ============================================================

void process_command(
    const char* command
)
{
    float v;
    float omega;

    if (
        sscanf(
            command,
            "VEL %f %f",
            &v,
            &omega
        ) == 2
    )
    {
        cmd_vel_to_ticks(
            v,
            omega,
            m0_target_ticks_s,
            m1_target_ticks_s
        );

        printf(
            "[CMD] "
            "v=%.3f m/s "
            "omega=%.3f rad/s\n",
            v,
            omega
        );

        printf(
            "      "
            "M0 target = %.2f ticks/s\n"
            "      "
            "M1 target = %.2f ticks/s\n",
            m0_target_ticks_s,
            m1_target_ticks_s
        );

        return;
    }

    if (
        strcmp(
            command,
            "STOP"
        ) == 0
    )
    {
        m0_target_ticks_s = 0.0f;
        m1_target_ticks_s = 0.0f;

        printf(
            "[CMD] STOP\n"
        );

        return;
    }

    // --------------------------------------------------------
// KP
//
// Example:
//   KP 2.5
// --------------------------------------------------------

float new_kp;

if (
    sscanf(
        command,
        "KP %f",
        &new_kp
    ) == 1
)
{
    new_kp =
        clamp_float(
            new_kp,
            KP_MIN,
            KP_MAX
        );

    control_kp =
        new_kp;

    printf(
        "[CMD] KP = %.3f\n",
        control_kp
    );

    return;
}


// --------------------------------------------------------
// DEAD
//
// Dead-zone / minimum running PWM.
//
// Example:
//   DEAD 40
//   DEAD 42.5
//
// Input unit is percent.
// --------------------------------------------------------

    float dead_percent;

    if (
        sscanf(
            command,
            "DEAD %f",
            &dead_percent
        ) == 1
    )
    {
        dead_percent =
            clamp_float(
                dead_percent,
                PWM_MIN_RUN_MIN / 10.0f,
                PWM_MIN_RUN_MAX / 10.0f
            );

        pwm_min_run =
            dead_percent *
            10.0f;

        printf(
            "[CMD] DEAD = %.1f %%\n",
            pwm_min_run / 10.0f
        );

        return;
    }

    if (
        strcmp(
            command,
            "DEBUG"
        ) == 0
    )
    {
        printf("[DEBUG]\n");

        printf(
            "Kp       = %.3f\n",
            control_kp
        );

        printf(
            "Dead PWM = %.1f %%\n",
            pwm_min_run / 10.0f
        );

        printf(
            "M0 T=%8.2f L=%8.2f RAW=%8.2f FIL=%8.2f "
            "ERR=%8.2f PWM=%6.1f%% "
            "START=%d ARMED=%d COAST=%d DIRWAIT=%d\n",

            m0_target_ticks_s,
            m0_limited_target,
            m0_actual_raw_ticks_s,
            m0_actual_ticks_s,
            m0_control_error,
            m0_pwm_command / 10.0f,

            m0_startup_active ? 1 : 0,
            m0_startup_armed ? 1 : 0,
            m0_coasting ? 1 : 0,
            m0_direction_change_wait ? 1 : 0
        );

        printf(
            "M1 T=%8.2f L=%8.2f RAW=%8.2f FIL=%8.2f "
            "ERR=%8.2f PWM=%6.1f%% "
            "START=%d ARMED=%d COAST=%d DIRWAIT=%d\n",

            m1_target_ticks_s,
            m1_limited_target,
            m1_actual_raw_ticks_s,
            m1_actual_ticks_s,
            m1_control_error,
            m1_pwm_command / 10.0f,

            m1_startup_active ? 1 : 0,
            m1_startup_armed ? 1 : 0,
            m1_coasting ? 1 : 0,
            m1_direction_change_wait ? 1 : 0
        );

        printf(
            "M0 pulses=%lu\n",
            static_cast<unsigned long>(
                fg_m0.pulse_count
            )
        );

        printf(
            "M1 pulses=%lu\n",
            static_cast<unsigned long>(
                fg_m1.pulse_count
            )
        );

        return;
    }

    if (command[0] == '\0')
    {
        return;
    }

    printf(
        "[ERR] Unknown command: %s\n",
        command
    );
}


// ============================================================
// UART COMMAND RECEIVER
// ============================================================

void process_uart_command()
{
    static char buffer[128];
    static uint index = 0;

    while (
        uart_is_readable(
            UPPER_UART
        )
    )
    {
        char c =
            uart_getc(
                UPPER_UART
            );

        if (
            c == '\n' ||
            c == '\r'
        )
        {
            if (index > 0)
            {
                buffer[index] = '\0';

                process_command(
                    buffer
                );

                index = 0;
            }
        }
        else
        {
            if (
                index <
                sizeof(buffer) - 1
            )
            {
                buffer[index++] =
                    c;
            }
        }
    }
}


// ============================================================
// USB COMMAND RECEIVER
// ============================================================

void process_usb_command()
{
    static char buffer[128];
    static uint index = 0;

    int c;

    while (
        (c = getchar_timeout_us(0))
        >= 0
    )
    {
        if (
            c == '\n' ||
            c == '\r'
        )
        {
            if (index > 0)
            {
                buffer[index] = '\0';

                process_command(
                    buffer
                );

                index = 0;
            }
        }
        else
        {
            if (
                index <
                sizeof(buffer) - 1
            )
            {
                buffer[index++] =
                    static_cast<char>(c);
            }
        }
    }
}


// ============================================================
// POWER PICO UART1 COMMAND PROCESSOR
// ============================================================
//
// STOP : latch immediate hard-stop
// RUN  : release hard-stop latch
//
// Unknown commands are ignored so that future power/status
// messages can coexist on UART1 without affecting motion.
// ============================================================

void process_power_command(
    const char* command
)
{
    if (
        strcmp(
            command,
            "STOP"
        ) == 0
    )
    {
        power_hard_stop_active = true;
        core1_hard_stop_request = true;

        // Core 0 target is also cleared immediately.
        m0_target_ticks_s = 0.0f;
        m1_target_ticks_s = 0.0f;

        return;
    }


    if (
        strcmp(
            command,
            "RUN"
        ) == 0
    )
    {
        // Release only. Do not restore the previous target.
        // A fresh /cmd_vel must arrive before motion resumes.
        power_hard_stop_active = false;
        core1_hard_stop_request = false;

        m0_target_ticks_s = 0.0f;
        m1_target_ticks_s = 0.0f;

        cmd_vel_received = false;
        cmd_vel_watchdog_active = true;
        last_cmd_vel_us = 0;

        return;
    }
}


// ============================================================
// POWER PICO UART1 RECEIVER
// ============================================================

void process_power_uart()
{
    static char buffer[64];
    static uint index = 0;

    while (
        uart_is_readable(
            POWER_UART
        )
    )
    {
        char c =
            uart_getc(
                POWER_UART
            );

        if (
            c == '\n' ||
            c == '\r'
        )
        {
            if (index > 0)
            {
                buffer[index] = '\0';

                process_power_command(
                    buffer
                );

                index = 0;
            }
        }
        else
        {
            if (
                index <
                sizeof(buffer) - 1
            )
            {
                buffer[index++] = c;
            }
        }
    }
}


// ============================================================
// CORE 1
// ============================================================

void core1_entry()
{
    setup_direction_pin(
        M0_DIR_PIN
    );

    setup_direction_pin(
        M1_DIR_PIN
    );

    setup_pwm(
        M0_PWM_PIN
    );

    setup_pwm(
        M1_PWM_PIN
    );


    float target_m0 = 0.0f;
    float target_m1 = 0.0f;

    float limited_target_m0 = 0.0f;
    float limited_target_m1 = 0.0f;


    int direction_m0 = 1;
    int direction_m1 = 1;


    bool startup_m0 = false;
    bool startup_m1 = false;

    // Startup is initially armed because boot state = STOP.
    bool startup_armed_m0 = true;
    bool startup_armed_m1 = true;

    uint64_t startup_start_m0 = 0;
    uint64_t startup_start_m1 = 0;

    // --------------------------------------------------------
    // Actual applied PWM state
    //
    // These variables are local to Core 1.
    // --------------------------------------------------------

    float current_pwm_m0 = 0.0f;
    float current_pwm_m1 = 0.0f;


    uint64_t previous_us =
        time_us_64();


    while (true)
    {
        // ====================================================
        // POWER PICO HARD-STOP
        // ====================================================
        //
        // This bypasses target ramping, startup logic and PWM
        // slew. Both PWM outputs are forced to zero immediately.
        // DIR pins are intentionally left unchanged.
        // ====================================================

        if (core1_hard_stop_request)
        {
            target_m0 = 0.0f;
            target_m1 = 0.0f;

            limited_target_m0 = 0.0f;
            limited_target_m1 = 0.0f;

            startup_m0 = false;
            startup_m1 = false;

            startup_armed_m0 = true;
            startup_armed_m1 = true;

            current_pwm_m0 = 0.0f;
            current_pwm_m1 = 0.0f;

            m0_pwm_command = 0.0f;
            m1_pwm_command = 0.0f;

            m0_limited_target = 0.0f;
            m1_limited_target = 0.0f;

            m0_control_error = 0.0f;
            m1_control_error = 0.0f;

            m0_startup_active = false;
            m1_startup_active = false;

            m0_startup_armed = true;
            m1_startup_armed = true;

            m0_coasting = true;
            m1_coasting = true;

            m0_direction_change_wait = false;
            m1_direction_change_wait = false;

            set_motor_output(
                M0_PWM_PIN,
                M0_DIR_PIN,
                0.0f,
                false
            );

            set_motor_output(
                M1_PWM_PIN,
                M1_DIR_PIN,
                0.0f,
                true
            );

            // Drain any queued pre-stop/zero target words so Core 0
            // can never block on a full multicore FIFO.
            while (multicore_fifo_rvalid())
            {
                (void) multicore_fifo_pop_blocking();
            }

            tight_loop_contents();
            continue;
        }


        // ====================================================
        // Receive target from Core 0
        // ====================================================

        while (
            multicore_fifo_rvalid()
        )
        {
            int32_t m0_x10 =
                static_cast<int32_t>(
                    multicore_fifo_pop_blocking()
                );

            if (
                multicore_fifo_rvalid()
            )
            {
                int32_t m1_x10 =
                    static_cast<int32_t>(
                        multicore_fifo_pop_blocking()
                    );

                target_m0 =
                    m0_x10 / 10.0f;

                target_m1 =
                    m1_x10 / 10.0f;
            }
        }


        // ====================================================
        // Fixed control period
        // ====================================================

        uint64_t now_us =
            time_us_64();

        uint64_t elapsed_us =
            now_us -
            previous_us;

        if (
            elapsed_us <
            CONTROL_PERIOD_US
        )
        {
            tight_loop_contents();
            continue;
        }

        previous_us =
            now_us;

        float dt =
            elapsed_us /
            1000000.0f;


        // ====================================================
        // ACCELERATION LIMIT
        // ====================================================

        float max_delta =
            MAX_ACCEL_TICKS_PER_SEC2 *
            dt;


        // M0
        if (
            limited_target_m0 <
            target_m0
        )
        {
            limited_target_m0 +=
                max_delta;

            if (
                limited_target_m0 >
                target_m0
            )
            {
                limited_target_m0 =
                    target_m0;
            }
        }
        else if (
            limited_target_m0 >
            target_m0
        )
        {
            limited_target_m0 -=
                max_delta;

            if (
                limited_target_m0 <
                target_m0
            )
            {
                limited_target_m0 =
                    target_m0;
            }
        }


        // M1
        if (
            limited_target_m1 <
            target_m1
        )
        {
            limited_target_m1 +=
                max_delta;

            if (
                limited_target_m1 >
                target_m1
            )
            {
                limited_target_m1 =
                    target_m1;
            }
        }
        else if (
            limited_target_m1 >
            target_m1
        )
        {
            limited_target_m1 -=
                max_delta;

            if (
                limited_target_m1 <
                target_m1
            )
            {
                limited_target_m1 =
                    target_m1;
            }
        }


        m0_limited_target =
            limited_target_m0;

        m1_limited_target =
            limited_target_m1;


        float actual_m0 =
            m0_actual_ticks_s;

        float actual_m1 =
            m1_actual_ticks_s;


        // ====================================================
        // M0 CONTROL
        // ====================================================

        {
            float target_abs =
                fabsf(limited_target_m0);

            float actual_abs =
                fabsf(actual_m0);

            float error =
                target_abs -
                actual_abs;

            m0_control_error =
                error;


            // ------------------------------------------------
            // STOP / COAST
            // ------------------------------------------------

            if (
                target_abs < 1.0f
            )
            {
                startup_m0 = false;

                startup_armed_m0 = true;

                m0_startup_active = false;
                m0_startup_armed = true;

                m0_coasting = true;
                m0_direction_change_wait = false;

                m0_control_error = 0.0f;

                // ------------------------------------------------
                // STOP = IMMEDIATE PWM OFF
                // ------------------------------------------------

                current_pwm_m0 = 0.0f;
                m0_pwm_command = 0.0f;

                set_motor_output(
                    M0_PWM_PIN,
                    M0_DIR_PIN,
                    0.0f,
                    false   // 현재 M0 방향 설정에 맞게
                );
            }


            // ------------------------------------------------
            // DIRECTION CHANGE
            // ------------------------------------------------

            else if (
                direction_m0 !=
                (
                    limited_target_m0 > 0.0f
                        ? 1
                        : -1
                )
            )
            {
                startup_m0 = false;

                m0_startup_active = false;
                m0_coasting = true;
                m0_direction_change_wait = true;


                // --------------------------------------------------------
                // Smoothly remove drive torque.
                // --------------------------------------------------------

                current_pwm_m0 =
                    slew_pwm(
                        current_pwm_m0,
                        0.0f,
                        dt
                    );

                m0_pwm_command =
                    current_pwm_m0;


                // --------------------------------------------------------
                // DIR may change ONLY after:
                //
                // 1. PWM has reached zero
                // 2. physical speed is sufficiently low
                // --------------------------------------------------------

                if (
                    current_pwm_m0 <=
                        PWM_ZERO_THRESHOLD &&
                    actual_abs <=
                        DIRECTION_CHANGE_SPEED
                )
                {
                    current_pwm_m0 = 0.0f;

                    direction_m0 =
                        limited_target_m0 > 0.0f
                            ? 1
                            : -1;

                    m0_direction_change_wait =
                        false;

                    startup_m0 = true;
                    startup_armed_m0 = false;

                    startup_start_m0 =
                        now_us;

                    m0_startup_active = true;
                    m0_startup_armed = false;

                    m0_coasting = false;
                }


                set_motor_output(
                    M0_PWM_PIN,
                    M0_DIR_PIN,
                    direction_m0 > 0
                        ? current_pwm_m0
                        : -current_pwm_m0,
                    true
                );
            }


            // ------------------------------------------------
            // MOVING
            // ------------------------------------------------

            else
            {
                // --------------------------------------------
                // ONE-SHOT STARTUP ENTRY
                // --------------------------------------------
                //
                // actual_abs < 5 alone is NOT sufficient.
                //
                // Startup must also be ARMED.
                // Once entered, it is immediately disarmed.
                // --------------------------------------------

                if (
                    !startup_m0 &&
                    startup_armed_m0 &&
                    actual_abs <
                    STARTUP_ENTRY_SPEED
                )
                {
                    startup_m0 = true;

                    startup_armed_m0 = false;

                    startup_start_m0 =
                        now_us;
                }


                // --------------------------------------------
                // STARTUP
                // --------------------------------------------

                if (startup_m0)
                {
                    uint64_t startup_elapsed_ms =
                        (
                            now_us -
                            startup_start_m0
                        ) / 1000;


                    m0_startup_active = true;
                    m0_startup_armed = false;

                    m0_coasting = false;
                    m0_direction_change_wait = false;


                    float startup_exit_speed =
                        fminf(
                            STARTUP_MAX_EXIT_SPEED,
                            target_abs *
                            STARTUP_EXIT_FRACTION
                        );


                    if (
                        actual_abs >=
                        startup_exit_speed
                    )
                    {
                        // Startup ends.
                        // DO NOT re-arm here.
                        //
                        // Re-arm occurs only at STOP.
                        startup_m0 = false;

                        m0_startup_active = false;
                    }
                    else if (
                        startup_elapsed_ms >=
                        STARTUP_TIMEOUT_MS
                    )
                    {
                        // Timeout also ends startup permanently
                        // for this run cycle.
                        startup_m0 = false;

                        m0_startup_active = false;

                        m0_pwm_command = 0.0f;
                        m0_coasting = true;

                        set_motor_output(
                            M0_PWM_PIN,
                            M0_DIR_PIN,
                            0.0f,
                            false
                        );
                    }
                    else
                    {
                        float output =
                            PWM_START_DUTY;

                        // Startup boost is immediate.
                        current_pwm_m0 =
                            output;

                        m0_pwm_command =
                            output;

                        set_motor_output(
                            M0_PWM_PIN,
                            M0_DIR_PIN,
                            direction_m0 > 0
                                ? output
                                : -output,
                            false
                        );
                    }
                }


                // --------------------------------------------
                // NORMAL SPEED CONTROL
                // --------------------------------------------

                else
                {
                    float kp =
                        control_kp;

                    float min_run =
                        pwm_min_run;


                    // ========================================================
                    // SPEED HYSTERESIS
                    // ========================================================
                    //
                    // error = target - actual
                    //
                    // error < -DB
                    //     -> request PWM reduction toward zero
                    //
                    // error > +DB
                    //     -> resume active drive
                    //
                    // inside deadband
                    //     -> preserve drive/coast state
                    // ========================================================

                    if (m0_coasting)
                    {
                        if (
                            error >
                            SPEED_DB_EXIT
                        )
                        {
                            m0_coasting = false;
                        }
                    }
                    else
                    {
                        if (
                            error <
                            -SPEED_DB_ENTER
                        )
                        {
                            m0_coasting = true;
                        }
                    }


                    float desired_pwm = 0.0f;


                    // --------------------------------------------------------
                    // DECELERATION
                    //
                    // Do NOT immediately cut PWM.
                    //
                    // desired = 0,
                    // actual PWM slowly approaches zero.
                    // --------------------------------------------------------

                    if (m0_coasting)
                    {
                        desired_pwm = pwm_min_run;
                    }


                    // --------------------------------------------------------
                    // DRIVE
                    // --------------------------------------------------------

                    else
                    {
                        float p_error =
                            error;

                        // Slight overspeed while still inside hysteresis
                        // must not create a negative PWM command.
                        if (p_error < 0.0f)
                        {
                            p_error = 0.0f;
                        }

                        float p_term =
                            kp *
                            p_error;

                        desired_pwm =
                            min_run +
                            p_term;

                        desired_pwm =
                            clamp_float(
                                desired_pwm,
                                pwm_min_run,
                                PWM_MAX_COMMAND
                            );
                    }


                    // ========================================================
                    // PWM SLEW LIMIT
                    // ========================================================

                    current_pwm_m0 =
                        slew_pwm(
                            current_pwm_m0,
                            desired_pwm,
                            dt
                        );


                    m0_pwm_command =
                        current_pwm_m0;

                    m0_direction_change_wait =
                        false;


                    set_motor_output(
                        M0_PWM_PIN,
                        M0_DIR_PIN,
                        direction_m0 > 0
                            ? current_pwm_m0
                            : -current_pwm_m0,
                        false
                    );
                }
            }
        }


        // ====================================================
        // M1 CONTROL
        // ====================================================

        {
            float target_abs =
                fabsf(limited_target_m1);

            float actual_abs =
                fabsf(actual_m1);

            float error =
                target_abs -
                actual_abs;

            m1_control_error =
                error;


            // ------------------------------------------------
            // STOP / COAST
            // ------------------------------------------------

            if (
                target_abs < 1.0f
            )
            {
                startup_m1 = false;

                startup_armed_m1 = true;

                m1_startup_active = false;
                m1_startup_armed = true;

                m1_coasting = true;
                m1_direction_change_wait = false;

                m1_control_error = 0.0f;

                current_pwm_m1 = 0.0f;
                m1_pwm_command = 0.0f;

                set_motor_output(
                    M1_PWM_PIN,
                    M1_DIR_PIN,
                    0.0f,
                    true
                );
            }


            // ------------------------------------------------
            // DIRECTION CHANGE
            // ------------------------------------------------

            else if (
                direction_m1 !=
                (
                    limited_target_m1 > 0.0f
                        ? 1
                        : -1
                )
            )
            {
                startup_m1 = false;

                m1_startup_active = false;
                m1_coasting = true;
                m1_direction_change_wait = true;


                // --------------------------------------------------------
                // Smoothly remove drive torque.
                // --------------------------------------------------------

                current_pwm_m1 =
                    slew_pwm(
                        current_pwm_m1,
                        0.0f,
                        dt
                    );

                m1_pwm_command =
                    current_pwm_m1;


                // --------------------------------------------------------
                // DIR may change ONLY after:
                //
                // 1. PWM has reached zero
                // 2. physical speed is sufficiently low
                // --------------------------------------------------------

                if (
                    current_pwm_m1 <=
                        PWM_ZERO_THRESHOLD &&
                    actual_abs <=
                        DIRECTION_CHANGE_SPEED
                )
                {
                    current_pwm_m1 = 0.0f;

                    direction_m1 =
                        limited_target_m1 > 0.0f
                            ? 1
                            : -1;

                    m1_direction_change_wait =
                        false;

                    startup_m1 = true;
                    startup_armed_m1 = false;

                    startup_start_m1 =
                        now_us;

                    m1_startup_active = true;
                    m1_startup_armed = false;

                    m1_coasting = false;
                }


                set_motor_output(
                    M1_PWM_PIN,
                    M1_DIR_PIN,
                    direction_m1 > 0
                        ? current_pwm_m1
                        : -current_pwm_m1,
                    false
                );
            }


            // ------------------------------------------------
            // MOVING
            // ------------------------------------------------

            else
            {
                // --------------------------------------------
                // ONE-SHOT STARTUP ENTRY
                // --------------------------------------------

                if (
                    !startup_m1 &&
                    startup_armed_m1 &&
                    actual_abs <
                    STARTUP_ENTRY_SPEED
                )
                {
                    startup_m1 = true;

                    startup_armed_m1 = false;

                    startup_start_m1 =
                        now_us;
                }


                // --------------------------------------------
                // STARTUP
                // --------------------------------------------

                if (startup_m1)
                {
                    uint64_t startup_elapsed_ms =
                        (
                            now_us -
                            startup_start_m1
                        ) / 1000;


                    m1_startup_active = true;
                    m1_startup_armed = false;

                    m1_coasting = false;
                    m1_direction_change_wait = false;


                    float startup_exit_speed =
                        fminf(
                            STARTUP_MAX_EXIT_SPEED,
                            target_abs *
                            STARTUP_EXIT_FRACTION
                        );


                    if (
                        actual_abs >=
                        startup_exit_speed
                    )
                    {
                        startup_m1 = false;

                        m1_startup_active = false;
                    }
                    else if (
                        startup_elapsed_ms >=
                        STARTUP_TIMEOUT_MS
                    )
                    {
                        startup_m1 = false;

                        m1_startup_active = false;

                        m1_pwm_command = 0.0f;
                        m1_coasting = true;

                        set_motor_output(
                            M1_PWM_PIN,
                            M1_DIR_PIN,
                            0.0f,
                            true
                        );
                    }
                    else
                    {
                        float output =
                            PWM_START_DUTY;

                        current_pwm_m1 =
                            output;

                        m1_pwm_command =
                            output;

                        set_motor_output(
                            M1_PWM_PIN,
                            M1_DIR_PIN,
                            direction_m1 > 0
                                ? output
                                : -output,
                            true
                        );
                    }
                }


                // --------------------------------------------
                // NORMAL SPEED CONTROL
                // --------------------------------------------

                else
                {
                    float kp =
                        control_kp;

                    float min_run =
                        pwm_min_run;


                    // ========================================================
                    // SPEED HYSTERESIS
                    // ========================================================

                    if (m1_coasting)
                    {
                        if (
                            error >
                            SPEED_DB_EXIT
                        )
                        {
                            m1_coasting = false;
                        }
                    }
                    else
                    {
                        if (
                            error <
                            -SPEED_DB_ENTER
                        )
                        {
                            m1_coasting = true;
                        }
                    }


                    float desired_pwm = 0.0f;


                    // --------------------------------------------------------
                    // DECELERATION
                    // --------------------------------------------------------

                    if (m1_coasting)
                    {
                        desired_pwm = pwm_min_run;
                    }


                    // --------------------------------------------------------
                    // DRIVE
                    // --------------------------------------------------------

                    else
                    {
                        float p_error =
                            error;

                        if (p_error < 0.0f)
                        {
                            p_error = 0.0f;
                        }

                        float p_term =
                            kp *
                            p_error;

                        desired_pwm =
                            min_run +
                            p_term;

                        desired_pwm =
                            clamp_float(
                                desired_pwm,
                                pwm_min_run,
                                PWM_MAX_COMMAND
                            );
                    }


                    // ========================================================
                    // PWM SLEW LIMIT
                    // ========================================================

                    current_pwm_m1 =
                        slew_pwm(
                            current_pwm_m1,
                            desired_pwm,
                            dt
                        );


                    m1_pwm_command =
                        current_pwm_m1;

                    m1_direction_change_wait =
                        false;


                    set_motor_output(
                        M1_PWM_PIN,
                        M1_DIR_PIN,
                        direction_m1 > 0
                            ? current_pwm_m1
                            : -current_pwm_m1,
                        true
                    );
                }
            }
        }


        // ====================================================
        // Publish debug state
        // ====================================================

        m0_startup_armed =
            startup_armed_m0;

        m1_startup_armed =
            startup_armed_m1;

        m0_direction_state =
            direction_m0;

        m1_direction_state =
            direction_m1;
    }
}


// ============================================================
// micro-ROS ENTITY LIFECYCLE
// ============================================================

static void force_ros_command_stop()
{
    // Never preserve an old motion command across an Agent loss.
    m0_target_ticks_s = 0.0f;
    m1_target_ticks_s = 0.0f;

    last_cmd_vel_us = 0;
    cmd_vel_received = false;
    cmd_vel_watchdog_active = true;
}


static void reset_ros_object_handles()
{
    ros_node =
        rcl_get_zero_initialized_node();

    cmd_vel_subscription =
        rcl_get_zero_initialized_subscription();

    odom_publisher =
        rcl_get_zero_initialized_publisher();

    ros_executor =
        rclc_executor_get_zero_initialized_executor();
}


static void destroy_ros_entities()
{
    force_ros_command_stop();

    // If the Agent has disappeared, entity destruction must not
    // wait for a remote acknowledgement.
    if (ros_support_initialized)
    {
        rmw_context_t* rmw_context =
            rcl_context_get_rmw_context(
                &ros_support.context
            );

        if (rmw_context != nullptr)
        {
            (void)
                rmw_uros_set_context_entity_destroy_session_timeout(
                    rmw_context,
                    0
                );
        }
    }

    if (ros_executor_initialized)
    {
        (void)rclc_executor_fini(
            &ros_executor
        );

        ros_executor_initialized = false;
    }

    if (ros_publisher_initialized)
    {
        (void)rcl_publisher_fini(
            &odom_publisher,
            &ros_node
        );

        ros_publisher_initialized = false;
    }

    if (ros_subscription_initialized)
    {
        (void)rcl_subscription_fini(
            &cmd_vel_subscription,
            &ros_node
        );

        ros_subscription_initialized = false;
    }

    if (ros_node_initialized)
    {
        (void)rcl_node_fini(
            &ros_node
        );

        ros_node_initialized = false;
    }

    if (ros_support_initialized)
    {
        (void)rclc_support_fini(
            &ros_support
        );

        ros_support_initialized = false;
    }

    if (odom_message_initialized)
    {
        nav_msgs__msg__Odometry__fini(
            &odom_msg
        );

        odom_message_initialized = false;
    }

    // Re-zero handles before the next creation cycle.
    ros_support = {};
    reset_ros_object_handles();

    // Do not integrate the disconnected time interval when ROS
    // reconnects. Pose itself is intentionally preserved.
    odom_previous_update_us = 0;
}


static bool create_ros_entities()
{
    // Always start from clean handles.
    destroy_ros_entities();

    ros_allocator =
        rcl_get_default_allocator();

    ros_support = {};
    reset_ros_object_handles();

    if (
        rclc_support_init(
            &ros_support,
            0,
            NULL,
            &ros_allocator
        ) != RCL_RET_OK
    )
    {
        destroy_ros_entities();
        return false;
    }

    ros_support_initialized = true;

    if (
        rclc_node_init_default(
            &ros_node,
            "cubic_c1_motor_controller",
            "",
            &ros_support
        ) != RCL_RET_OK
    )
    {
        destroy_ros_entities();
        return false;
    }

    ros_node_initialized = true;

    if (
        rclc_subscription_init_default(
            &cmd_vel_subscription,
            &ros_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                geometry_msgs,
                msg,
                Twist
            ),
            "/cmd_vel"
        ) != RCL_RET_OK
    )
    {
        destroy_ros_entities();
        return false;
    }

    ros_subscription_initialized = true;

    if (
        rclc_publisher_init_default(
            &odom_publisher,
            &ros_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                nav_msgs,
                msg,
                Odometry
            ),
            "/odom"
        ) != RCL_RET_OK
    )
    {
        destroy_ros_entities();
        return false;
    }

    ros_publisher_initialized = true;

    if (!nav_msgs__msg__Odometry__init(&odom_msg))
    {
        destroy_ros_entities();
        return false;
    }

    odom_message_initialized = true;

    if (
        !rosidl_runtime_c__String__assign(
            &odom_msg.header.frame_id,
            "odom"
        ) ||
        !rosidl_runtime_c__String__assign(
            &odom_msg.child_frame_id,
            "base_link"
        )
    )
    {
        destroy_ros_entities();
        return false;
    }

    if (
        rclc_executor_init(
            &ros_executor,
            &ros_support.context,
            1,
            &ros_allocator
        ) != RCL_RET_OK
    )
    {
        destroy_ros_entities();
        return false;
    }

    ros_executor_initialized = true;

    if (
        rclc_executor_add_subscription(
            &ros_executor,
            &cmd_vel_subscription,
            &cmd_vel_msg,
            &cmd_vel_callback,
            ON_NEW_DATA
        ) != RCL_RET_OK
    )
    {
        destroy_ros_entities();
        return false;
    }

    // Non-fatal. /odom has an uptime fallback if sync fails.
    (void)rmw_uros_sync_session(1000);

    // Require a fresh command after every connection/reconnection.
    force_ros_command_stop();

    // Avoid an odometry jump caused by disconnected wall time.
    odom_previous_update_us = 0;

    return true;
}


// ============================================================
// MAIN - CORE 0
// ============================================================

int main()
{
    // --------------------------------------------------------
    // micro-ROS transport
    //
    // pico_serial_transport_* is provided by
    // micro_ros_raspberrypi_pico_sdk/pico_uart_transport.c.
    //
    // IMPORTANT:
    // Do not printf() to the same stdio transport while
    // micro-ROS is running. It would corrupt the XRCE-DDS
    // serial stream.
    // --------------------------------------------------------

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );


    // --------------------------------------------------------
    // UART1 - Power Pico
    // --------------------------------------------------------

    uart_init(
        POWER_UART,
        UART_BAUD
    );

    gpio_set_function(
        UART1_TX_PIN,
        GPIO_FUNC_UART
    );

    gpio_set_function(
        UART1_RX_PIN,
        GPIO_FUNC_UART
    );


    // --------------------------------------------------------
    // FG acquisition
    // --------------------------------------------------------

    setup_adc();
    setup_adc_dma();


    // --------------------------------------------------------
    // Core 1 motor-control loop
    // --------------------------------------------------------

    multicore_launch_core1(
        core1_entry
    );


    // --------------------------------------------------------
    // Safe boot state
    // --------------------------------------------------------

    m0_target_ticks_s = 0.0f;
    m1_target_ticks_s = 0.0f;

    last_cmd_vel_us = 0;
    cmd_vel_received = false;
    cmd_vel_watchdog_active = true;

    power_hard_stop_active = false;
    core1_hard_stop_request = false;


    // --------------------------------------------------------
    // micro-ROS reconnect state machine starts in WAITING_AGENT.
    //
    // Do NOT block here waiting for the SBC. Core 0 must keep
    // processing safety, power UART, ADC/DMA and motor targets.
    // --------------------------------------------------------

    micro_ros_state =
        MicroRosState::WAITING_AGENT;

    last_ros_agent_check_us = 0;
    ros_agent_miss_count = 0;

    ros_allocator =
        rcl_get_default_allocator();

    ros_support = {};
    reset_ros_object_handles();

    force_ros_command_stop();


    uint64_t last_fifo_update_us = 0;


    while (true)
    {
        // ====================================================
        // micro-ROS reconnect state machine
        // ====================================================

        uint64_t ros_now_us =
            time_us_64();

        switch (micro_ros_state)
        {
            // ------------------------------------------------
            // Agent is not available yet.
            // ------------------------------------------------
            case MicroRosState::WAITING_AGENT:
            {
                force_ros_command_stop();

                if (
                    last_ros_agent_check_us == 0 ||
                    ros_now_us -
                    last_ros_agent_check_us >=
                    static_cast<uint64_t>(
                        ROS_WAIT_PING_INTERVAL_MS
                    ) * 1000ULL
                )
                {
                    last_ros_agent_check_us =
                        ros_now_us;

                    if (
                        rmw_uros_ping_agent(
                            ROS_PING_TIMEOUT_MS,
                            1
                        ) == RCL_RET_OK
                    )
                    {
                        micro_ros_state =
                            MicroRosState::AGENT_AVAILABLE;
                    }
                }

                break;
            }

            // ------------------------------------------------
            // Agent answered. Build ROS entities.
            // ------------------------------------------------
            case MicroRosState::AGENT_AVAILABLE:
            {
                if (create_ros_entities())
                {
                    ros_agent_miss_count = 0;

                    last_ros_agent_check_us =
                        time_us_64();

                    micro_ros_state =
                        MicroRosState::AGENT_CONNECTED;
                }
                else
                {
                    destroy_ros_entities();

                    last_ros_agent_check_us =
                        time_us_64();

                    micro_ros_state =
                        MicroRosState::WAITING_AGENT;
                }

                break;
            }

            // ------------------------------------------------
            // Normal ROS operation.
            // ------------------------------------------------
            case MicroRosState::AGENT_CONNECTED:
            {
                (void)rclc_executor_spin_some(
                    &ros_executor,
                    RCL_MS_TO_NS(1)
                );

                if (
                    ros_now_us -
                    last_ros_agent_check_us >=
                    static_cast<uint64_t>(
                        ROS_CONNECTED_PING_INTERVAL_MS
                    ) * 1000ULL
                )
                {
                    last_ros_agent_check_us =
                        ros_now_us;

                    if (
                        rmw_uros_ping_agent(
                            ROS_PING_TIMEOUT_MS,
                            1
                        ) == RCL_RET_OK
                    )
                    {
                        ros_agent_miss_count = 0;
                    }
                    else
                    {
                        if (
                            ros_agent_miss_count <
                            0xFF
                        )
                        {
                            ros_agent_miss_count++;
                        }

                        if (
                            ros_agent_miss_count >=
                            ROS_DISCONNECT_MISSES
                        )
                        {
                            micro_ros_state =
                                MicroRosState::AGENT_DISCONNECTED;
                        }
                    }
                }

                break;
            }

            // ------------------------------------------------
            // Lost Agent. Remove stale ROS state and retry.
            // ------------------------------------------------
            case MicroRosState::AGENT_DISCONNECTED:
            {
                force_ros_command_stop();

                destroy_ros_entities();

                ros_agent_miss_count = 0;

                last_ros_agent_check_us =
                    time_us_64();

                micro_ros_state =
                    MicroRosState::WAITING_AGENT;

                break;
            }
        }


        // ====================================================
        // UART1 - Power Pico
        // ====================================================

        process_power_uart();


        // ====================================================
        // ADC / FG acquisition
        // ====================================================
        //
        // Sampling and FG edge processing run continuously through
        // ping-pong DMA + DMA IRQ 0. No polling/restart gap here.
        // ====================================================


        // ====================================================
        // FG speed
        // ====================================================

        update_fg_speed();


        uint64_t now_us =
            time_us_64();


        // ====================================================
        // Wheel odometry -> /odom
        // ====================================================

        if (
            micro_ros_state ==
            MicroRosState::AGENT_CONNECTED
        )
        {
            update_and_publish_odometry(
                now_us
            );
        }


        // ====================================================
        // Software cmd_vel watchdog
        // ====================================================

        update_cmd_vel_watchdog(
            now_us
        );


        // ====================================================
        // FINAL POWER HARD-STOP OVERRIDE
        // ====================================================

        if (power_hard_stop_active)
        {
            m0_target_ticks_s = 0.0f;
            m1_target_ticks_s = 0.0f;
            core1_hard_stop_request = true;
        }


        // ====================================================
        // Core 0 -> Core 1
        // ====================================================

        if (
            !power_hard_stop_active &&
            now_us -
            last_fifo_update_us
            >= 5000
        )
        {
            int32_t m0_x10 =
                static_cast<int32_t>(
                    m0_target_ticks_s *
                    10.0f
                );

            int32_t m1_x10 =
                static_cast<int32_t>(
                    m1_target_ticks_s *
                    10.0f
                );

            multicore_fifo_push_blocking(
                static_cast<uint32_t>(
                    m0_x10
                )
            );

            multicore_fifo_push_blocking(
                static_cast<uint32_t>(
                    m1_x10
                )
            );

            last_fifo_update_us =
                now_us;
        }


        tight_loop_contents();
    }


    return 0;
}