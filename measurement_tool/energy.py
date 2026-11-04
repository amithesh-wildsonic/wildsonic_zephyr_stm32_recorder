from __future__ import annotations

"""Energy calculation and unit conversion utilities.

Physics reference:
- Power (mW) * Duration (s) = Energy (mJ)      [10^-3 W * s = 10^-3 J]
- Power (W)  * Duration (s) = Energy (J)       [1 W * s = 1 J]
- Power (uW) * Duration (s) = Energy (uJ)      [10^-6 W * s = 10^-6 J]
- Voltage (V) * Current (mA) = Power (mW)
- Voltage (V) * Current (A)  = Power (W)
- Voltage (V) * Current (uA) = Power (uW)
"""


def power_mw_from_v_ma(voltage_v: float, current_ma: float) -> float:
    """Calculates power in mW from voltage in V and current in mA."""
    return float(voltage_v) * float(current_ma)


def energy_mj_from_mw_s(power_mw: float, duration_s: float) -> float:
    """Calculates energy in mJ from power in mW and duration in seconds.

    1 mW * 1 s = 1 mJ (milliwatt-second = millijoule).
    """
    return float(power_mw) * max(0.0, float(duration_s))


def energy_j_from_w_s(power_w: float, duration_s: float) -> float:
    """Calculates energy in J from power in W and duration in seconds."""
    return float(power_w) * max(0.0, float(duration_s))


def energy_uj_from_uw_s(power_uw: float, duration_s: float) -> float:
    """Calculates energy in uJ from power in uW and duration in seconds."""
    return float(power_uw) * max(0.0, float(duration_s))


def convert_energy(value: float, from_unit: str, to_unit: str) -> float:
    """Converts energy between J, mJ, uJ, and Wh / mWh."""
    u_from = from_unit.strip().lower()
    u_to = to_unit.strip().lower()

    # Convert to standard Joules
    to_joules = {
        "j": 1.0,
        "joules": 1.0,
        "joule": 1.0,
        "mj": 1e-3,
        "millijoules": 1e-3,
        "millijoule": 1e-3,
        "uj": 1e-6,
        "microjoules": 1e-6,
        "microjoule": 1e-6,
        "wh": 3600.0,
        "watt-hours": 3600.0,
        "mwh": 3.6,
        "milliwatt-hours": 3.6,
        "uwh": 3.6e-3,
    }

    if u_from not in to_joules:
        raise ValueError(f"Unsupported source energy unit: '{from_unit}'")
    if u_to not in to_joules:
        raise ValueError(f"Unsupported target energy unit: '{to_unit}'")

    joules = float(value) * to_joules[u_from]
    return joules / to_joules[u_to]

