# fpga-robot-asic

Soft-core RISC-V SoC on FPGA for real-time robotic manipulation with a production-style UVM verification environment and a clean migration path to ASIC.

> goals
- 1 kHz control loop (fixed tick)  
- ≤ 50 µs worst-case sensor-to-actuator latency  
- ≤ 10 µs PWM edge jitter (12 channels)  
- Deterministic DMA that cannot starve control path  
- Firmware-in-the-loop (FIL) + UVM coverage before HIL

---

## 0. tl;dr (quick start)

# deps (Ubuntu-ish)
sudo apt-get install make gcc g++ python3 python3-venv git graphviz
# riscv toolchain (recommended): https://github.com/riscv-collab/riscv-gnu-toolchain
# verilator (optional for open sim): https://www.veripool.org/verilator/

# clone
git clone https://example.com/fpga-robot-asic.git
cd fpga-robot-asic

# build firmware (bare-metal)
make sw

# fast open-source sim (sanity; no full UVM)
make sim-verilator

# full DV with UVM (Questa/VCS/etc. required; set tools in env.mk)
make dv-regress

# synthesize for DE1-SoC (Cyclone V) with Quartus in PATH
make fpga BOARD=de1soc

# program board (example; adjust cable index)
make prog BOARD=de1soc

# run hardware-in-the-loop tests (serial @ /dev/ttyUSB0)
make hil PORT=/dev/ttyUSB0
1. what this is
A minimal but complete HW/SW stack for robotic hands/arms:

RV32IMC soft core, AXI-Lite interconnect

12-channel PWM/timer block (1 µs resolution, double-buffered)

SPI + I²C (DMA-assisted), timestamped IRQs

Fixed-point DSP accelerator (PID; tiny-CNN int8 mode)

Bare-metal/RTOS firmware with strict 1 kHz schedule

UVM testbench (agents, scoreboard, SVA, coverage) + FIL

HIL bench scripts for step/chirp/grasp scenarios

ASIC notes: DFT/MBIST hooks, CDC, clock gating

2. repo layout
bash
Copy code
rtl/                 # synthesizable RTL
  cpu/               # RV32I[M][C] microcore (2-3 stage)
  bus/               # AXI-lite, simple stream
  periph/            # pwm, spi, i2c, uart, timers
  accel/             # fixed-point pid / tiny-cnn
  top/               # fpga top, board pinouts

dv/                  # verification
  uvm/               # agents, sequences, scoreboard, env, tests
  sva/               # assertions (latency, jitter, starvation)
  fil/               # firmware-in-the-loop (ELF loader, co-sim glue)

sw/                  # firmware
  hal/               # MMIO register drivers
  apps/              # control loop, telemetry
  rtos/              # optional FreeRTOS port
  scripts/           # log parsers/plots

fpga/                # synth flows
  de1soc/            # Quartus project files, SDC
  artix7/            # Vivado alt flow (optional)

hil/                 # test rigs, serial tools, IMU profiles
doc/                 # diagrams, register maps, notes

env.mk               # tool config (edit this)
Makefile             # unified entry
3. architecture (short)
sql
Copy code
+------------------------ SoC (single clock, 50–100 MHz) -----------------------+
|   RV32IMC CPU  -- AXI-Lite --  PWM(12ch)   SPI   I2C   UART   DSP-ACCEL       |
|        |                 \        |        |     |      |        ^            |
|     SRAM 64–128KB        +-- DMA -+--------+-----+------+--------+            |
|        |                         timestamped IRQs             AXI-stream     |
+------------------------------------------------------------------------------+
external: 6-axis IMU / tactile sensors (SPI/I2C), 5–12 servos (PWM)
PWM: double-buffered compare; “safe window” signal for glitch-free updates

DSP: Q15.16 fixed-point PID; optional int8 1D/2D conv (3×3/1×1)

DMA: time-sliced; grants cannot preempt control path beyond bound M

4. register map (excerpt)
block	base	regs (offset)	notes
PWM	0x0001_0000	CTRL(0x00) PERIOD(0x04) CMP0..11(0x10..0x3C) STAT	double-buffer + window
I2C0	0x0002_0000	CTRL ADDR TXRX IRQ DMA	IRQ timestamps
SPI0	0x0002_4000	CTRL CS CLK TXRX IRQ DMA	mode 0/3
DSPA	0x0003_0000	MODE COEF_Kp/Ki/Kd IN OUT STAT	PID/CNN switch
TMR	0x0000_F000	CYCLE(0x00) TICKMS(0x04)	free-running counter

5. firmware (bare-metal default)
scheduling: 1 kHz control at highest priority; telemetry ≤ 50 Hz

numerics: Q15.16 fixed-point; saturation + anti-windup

safety: command watchdog; joint soft limits; graceful decel on fault

c
Copy code
// sw/apps/control/main.c
void control_tick(void) {
  sensor_t s = imu_get();
  for (int j=0;j<NJ;++j) {
    int32_t e = target[j] - s.pos[j];
    int32_t u = dsp_pid_step(j, e, s.vel[j]);  // bounded-latency accelerator call
    pwm_set(j, u);
  }
  telemetry_maybe_emit();
}
build:

bash
Copy code
make sw            # produces out/sw/firmware.elf(.bin)
6. verification (uvm + fil)
6.1 environment
agents: AXI-Lite, SPI/I²C, PWM-mon, IRQ-mon

scoreboard: MMIO intents → expected PWM updates (golden C model)

coverage: functional (interrupt phase vs PWM period, DMA sizes, saturations), code/toggle

FIL: run same firmware ELF on ISS/verilated core; buses driven by UVM

6.2 key assertions (sva)
systemverilog
Copy code
// no starvation: control write serviced promptly
property p_ctrl_write_serviced;
  @(posedge clk) disable iff (!rst_n)
  (ctrl_req && !dma_quiet) |-> ##[1:8] ctrl_grant;
endproperty
assert property (p_ctrl_write_serviced);

// glitch-free PWM updates (no mid-period glitch)
property p_pwm_glitch_free;
  @(posedge clk) disable iff (!rst_n)
  (update_pending && !safe_window) |-> nexttime(!pwm_out_change);
endproperty
assert property (p_pwm_glitch_free);

// bounded IMU IRQ -> PWM write latency
property p_irq2pwm_latency;
  @(posedge clk) disable iff (!rst_n)
  (imu_irq && enable) |-> ##[1:L_MAX] (pwm_write && same_tick);
endproperty
run:

bash
Copy code
# edit env.mk to point to your simulator (Questa/VCS/etc.)
make dv-regress
open-source sanity (limited UVM):

bash
Copy code
make sim-verilator
7. fpga build & program
7.1 de1-soc (cyclone v / quartus)
bash
Copy code
# Quartus in PATH; see fpga/de1soc/*.qsf for pins
make fpga BOARD=de1soc
make prog BOARD=de1soc
7.2 artix-7 (vivado; optional)
bash
Copy code
make fpga BOARD=artix7
make prog BOARD=artix7
constraints: see fpga/<board>/*.sdc|*.xdc
clock: 50–100 MHz single-domain preferred

8. hardware-in-the-loop (hil)
servos: connect 5–12 PWM outputs + external power (do not power servos from FPGA board 5V)

sensors: IMU (SPI or I²C), optional fingertip force sensors

serial: UART → PC for logs

run:

bash
Copy code
# record step/chirp, auto-analyze overshoot/settling
make hil PORT=/dev/ttyUSB0
python3 hil/step_test.py --joint 2 --amp 10
expected:

stable 1 kHz tick; no missed deadlines

≤ 5% overshoot with tuned PID; safe-stop on IMU dropout > 5 ms

9. performance targets (how to measure)
IRQ→PWM latency: enable a logic analyzer; toggle a GPIO in IRQ handler entry and on PWM compare write; worst-case ≤ 50 µs

PWM jitter: feed one channel into timebase; measure p-p across 1000 periods; ≤ 10 µs

throughput: IMU 1 kHz + tactile 500 Hz; DMA bursts ≤ 16 words; zero deadline misses @ 100 MHz

sw/scripts/analyze.py will parse serial logs and emit CSV + plots.

10. asic migration notes
memories: swap inferred RAMs with foundry SRAM macros; add MBIST

dft: full scan; expose JTAG TAP; boundary-scan pads

power: clock gating enables on CPU/peripherals/accelerator; domain split optional

timing: SS/125°C/Vmin; keep control path one-cycle AXI-Lite

verification reuse: same UVM env; FIL on gate-level (SDF) for key tests

bring-up: ROM diags → PWM loopback → SPI/I²C loopback → watchdog

11. configuration
edit env.mk to point to tools:

QUESTA_BIN, VCS_BIN (if available)

VERILATOR_BIN (optional)

QUARTUS_BIN or VIVADO_BIN

RISCV_PREFIX (e.g., riscv32-unknown-elf-)

board pinouts: fpga/<board>/pins.csv

12. known issues / todo
Verilator path runs limited checks (no full UVM semantics)

Tiny-CNN accelerator provides fixed kernels/sizes (see rtl/accel/params.svh)

UART micro-ROS bridge is optional; disabled by default

More formal proofs for safety properties desirable

13. minimal citations (for README)
Accellera UVM 1.2 / IEEE 1800.2-2020 for verification structure

RISC-V unprivileged spec for RV32IMC compliance

Classical control (PID; jitter/latency effects)

(Include full references in doc/refs.bib if publishing.)

