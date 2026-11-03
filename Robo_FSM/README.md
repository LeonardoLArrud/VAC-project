# Robo_FSM: FSM line follower on the real robot

The Pi runs the FSM (IDLE, FOLLOW, WAIT, SEARCH, TURN) and the pure pursuit follower.
The Arduino only applies the PWM it receives and reports the ultrasound distance.

| Folder | What it has |
|---|---|
| `pi/` | `fsm.py` (FSM and main loop), `line_follower.py`, `robot_hardware.py` (Pi Camera + serial) |
| `arduino/robo_fsm/` | `robo_fsm.ino` and the course files it uses (`Motor`, `MyServo`, `Parameters.h`) |

## Serial protocol (115200)

- Pi to Arduino: `S<left>,<right>` wheel speed in mm/s (per-wheel PI on the encoders), `M<left>,<right>` raw PWM -255..255, `X` stop. No command for 300 ms: motors stop.
- Arduino to Pi: `D,<cm>,<nFR>,<nFL>,<nRR>,<nRL>` every 50 ms (cm: -1 = no echo, -2 = no signal; n = encoder counts).

## Run

1. Upload `arduino/robo_fsm/robo_fsm.ino` (Bluno Mega 2560).
2. Copy `pi/` to the Pi (needs `python3-picamera2`, `python3-serial`, `numpy`).
3. On the Pi: `python3 robot_hardware.py`. Starts 1 s after launch, Ctrl+C stops. Writes `fsm_log.csv`.

## Before the first test

- `robot_hardware.py`: `PORT`, `CAM_FOV` (62.2 deg for camera v2, 53.5 deg for v1.3), `PWM_MIN`, `V_MAX`.
- `line_follower.py`: camera geometry `H_CAM`, `X_CAM`, `TILT`; `K`; `V0` (start low, e.g. 0.08); `THRESHOLD` for the lighting.
- `USE_ENCODERS` in `robot_hardware.py`: True = per-wheel PI and heading from the encoders; False = open-loop PWM.
- Checks: `python3 diag.py encoders` (counts go up when each wheel turns forward), then `python3 diag.py pi` (robot suspended, each wheel near 0.10 m/s).
  PI gains `KP`, `KI` and feedforward `U0`, `KFF` at the top of `robo_fsm.ino`; `ENC_SIGN` flips an encoder that counts backwards.
