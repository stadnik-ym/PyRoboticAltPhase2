#!/usr/bin/env python3

from smbus import SMBus
import time
import os

ADDR = 0x2D

CHARGE_STATES = {
    0: "standby",
    1: "trickle charge",
    2: "constant current charge",
    3: "constant voltage charge",
    4: "charging pending",
    5: "full",
    6: "charge timeout",
}


def read_u8(bus, reg):
    return bus.read_byte_data(ADDR, reg)


def read_u16(bus, low_reg):
    low = read_u8(bus, low_reg)
    high = read_u8(bus, low_reg + 1)
    return (high << 8) | low


def to_i16(value):
    if value & 0x8000:
        return value - 65536
    return value


def yes_no(value):
    return "YES" if value else "NO"


def minutes_to_text(minutes):
    if minutes <= 0 or minutes >= 65535:
        return "unknown"
    h = minutes // 60
    m = minutes % 60
    if h > 0:
        return f"{h}h {m}m"
    return f"{m}m"


try:
    bus = SMBus(1)

    while True:
        # --- Service/status registers ---
        device_id = read_u8(bus, 0x00)
        charge_reg = read_u8(bus, 0x02)
        comm_reg = read_u8(bus, 0x03)
        sw_rev = read_u8(bus, 0x50)

        is_charging = bool(charge_reg & 0x80)
        is_fast_charging = bool(charge_reg & 0x40)
        vbus_powered = bool(charge_reg & 0x20)
        charge_state_code = charge_reg & 0x07
        charge_state = CHARGE_STATES.get(charge_state_code, "unknown")

        bq4050_ok = bool(comm_reg & 0x02)
        ip2368_ok = bool(comm_reg & 0x01)

        # --- Type-C VBUS ---
        vbus_voltage_mv = read_u16(bus, 0x10)
        vbus_current_ma = to_i16(read_u16(bus, 0x12))
        vbus_power_mw = read_u16(bus, 0x14)

        # --- Battery pack ---
        battery_voltage_mv = read_u16(bus, 0x20)
        battery_current_ma = to_i16(read_u16(bus, 0x22))
        battery_percent = read_u16(bus, 0x24)
        remaining_capacity_mah = read_u16(bus, 0x26)
        remaining_discharge_min = read_u16(bus, 0x28)
        remaining_charge_min = read_u16(bus, 0x2A)

        # --- Individual cells ---
        cell1_mv = read_u16(bus, 0x30)
        cell2_mv = read_u16(bus, 0x32)
        cell3_mv = read_u16(bus, 0x34)
        cell4_mv = read_u16(bus, 0x36)

        cells = [cell1_mv, cell2_mv, cell3_mv, cell4_mv]
        cell_min = min(cells)
        cell_max = max(cells)
        cell_delta = cell_max - cell_min

        if battery_current_ma > 0:
            battery_mode = "CHARGING"
        elif battery_current_ma < 0:
            battery_mode = "DISCHARGING"
        else:
            battery_mode = "IDLE"

        os.system("clear")

        print("========== WAVESHARE UPS HAT (E) ==========")
        print(f"I2C address:          0x{ADDR:02X}")
        print(f"Device ID:            0x{device_id:02X}")
        print(f"Software revision:    {sw_rev}")
        print()

        print("--------------- STATUS --------------------")
        print(f"VBUS powered:         {yes_no(vbus_powered)}")
        print(f"Charging:             {yes_no(is_charging)}")
        print(f"Fast charging:        {yes_no(is_fast_charging)}")
        print(f"Charge state:         {charge_state}")
        print(f"Battery mode:         {battery_mode}")
        print(f"BQ4050 comm:          {'OK' if bq4050_ok else 'ERROR'}")
        print(f"IP2368 comm:          {'OK' if ip2368_ok else 'ERROR'}")
        print()

        print("--------------- BATTERY -------------------")
        print(f"Charge:               {battery_percent} %")
        print(f"Battery voltage:      {battery_voltage_mv / 1000:.3f} V")
        print(f"Battery current:      {battery_current_ma} mA")
        print(f"Battery power:        {(battery_voltage_mv * battery_current_ma) / 1_000_000:.2f} W")
        print(f"Remaining capacity:   {remaining_capacity_mah} mAh")
        print(f"Time to empty:        {minutes_to_text(remaining_discharge_min)}")
        print(f"Time to full:         {minutes_to_text(remaining_charge_min)}")
        print()

        print("--------------- CELLS ---------------------")
        print(f"Cell 1:               {cell1_mv / 1000:.3f} V")
        print(f"Cell 2:               {cell2_mv / 1000:.3f} V")
        print(f"Cell 3:               {cell3_mv / 1000:.3f} V")
        print(f"Cell 4:               {cell4_mv / 1000:.3f} V")
        print(f"Cell delta:           {cell_delta} mV")
        print()

        print("--------------- TYPE-C VBUS ---------------")
        print(f"VBUS voltage:         {vbus_voltage_mv / 1000:.3f} V")
        print(f"VBUS current:         {vbus_current_ma} mA")
        print(f"VBUS power:           {vbus_power_mw / 1000:.2f} W")
        print()

        print("Ctrl+C to exit")
        time.sleep(1)

except KeyboardInterrupt:
    print("\nОстановлено пользователем")

except Exception as e:
    print("Ошибка чтения UPS HAT по I2C")
    print(e)

finally:
    try:
        bus.close()
    except:
        pass
