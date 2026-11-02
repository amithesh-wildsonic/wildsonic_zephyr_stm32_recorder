# WildSonic BLE Integration & Hardware Transport Specification

**Target Hardware:** STM32L476RG  
**BLE Module:** HM-10 Compatible Bluetooth Low Energy UART Module  
**Interface:** USART1 (TX: PB6, RX: PB7) @ 9600 baud, 8 data bits, no parity, 1 stop bit  
**RTOS:** Zephyr 4.5.0-rc2  

---

## 1. Hardware Pinout & UART Interface

The STM32L476RG interfaces to the onboard BLE module using standard asynchronous serial communication:
- **MCU TX (PB6):** Transmits AT commands and telemetry responses to the BLE module.
- **MCU RX (PB7):** Receives mobile application commands and modem status notifications via interrupt-driven ring buffer.
- **Baud Rate:** 9600 bps (matched to default transparent bridge baud rate).
- **Interrupt Servicing:** Handled by Zephyr UART interrupt driver with non-blocking byte ingestion into a 512-byte ring buffer (`s_ble_rx_rb`).

---

## 2. Low-Power Architecture & Sleep Lifecycle

The BLE subsystem is integrated into the recorder's low-power state machine:

### 2.1 BLE Module Power Modes
- **Normal Transparent Mode (`WS0`):** Transparently forwards serial data between central and MCU.
- **Light Sleep Mode (`AT+SLPM1`):** Module enters light low-power sleep; wakes instantly upon serial RX or central connection.
- **Deep Sleep Command (`AT+SLEEP`):** Final provisioning command sent to power down radio when not connected.

### 2.2 Wake & Sleep Decoupling
- **RTC Stop 2 Sleep:** When the MCU prepares to enter Stop 2 sleep between recording windows, the BLE module is transitioned to low-power mode.
- **Stop 2 Resume:** Upon waking from Stop 2, no spurious AT transmissions occur on the UART bus. The BLE module is kept decoupled until central activity or scheduling requires active transmission.

---

## 3. Pairing Window & Connection Management

To preserve security and minimize idle power consumption, pairing and connection windows are actively managed:

1. **Boot / Wake Pairing Window:**
   On cold boot or wake event, the BLE module opens a 300-second connectable window.
2. **Heartbeat / Activity Refresh (`WS+ALIVE`):**
   While a mobile device remains connected and transmits `WS+ALIVE` or configuration queries, the connection remains active.
3. **Automatic Disconnect Locking:**
   When the mobile device disconnects (`OK+LOST`), if the pairing window has elapsed, the BLE interface automatically submits `s_ble_lock_work` to lock pairing.
4. **Manual Pairing Control:**
   Console command `BLEPAIR <seconds>` opens or extends the pairing window; `BLEPAIR LOCK` or `BLEPAIR 0` locks the module immediately.

---

## 4. Non-Blocking Buffers & Threading Architecture

The BLE communications stack is designed to avoid interfering with hard-real-time audio acquisition:

```
    [USART1 RX IRQ] ---> [512-Byte RX Ring Buffer]
                                   |
                                   v
                      [BLE Comm Thread (Priority 8)]
                                   |
                   +---------------+---------------+
                   |                               |
              [Modem ACK]                   [Command Line]
                   |                               |
             [Semaphore]              [Console Process Line]
                                                   |
                                                   v
    [USART1 TX] <--- [1024-Byte TX Ring Buffer] <--+
```

- **BLE Communication Thread:** Runs at priority 8 with a 3072-byte stack.
- **TX Queueing:** Outgoing data is buffered in a 1024-byte ring buffer (`s_ble_tx_rb`) and drained asynchronously, preventing thread starvation during large device info dumps.
- **Frame Assembly:** Protects against partial line fragments and buffer overrun (lines > 128 bytes are safely truncated to avoid stack corruption).

---

## 5. Deferred Identity Updates

When a user modifies the advertised device name (`DNAME`) or serial number (`SN`) over BLE while connected:
1. New values are validated and stored in RAM.
2. The BLE module cannot re-advertise while a connection is established. The update is flagged as `s_name_update_pending = true`.
3. Upon disconnect (`OK+LOST`), the supervisor automatically applies the new name and serial number to the module using `AT+NAME<DNAME>-<SN>`.

