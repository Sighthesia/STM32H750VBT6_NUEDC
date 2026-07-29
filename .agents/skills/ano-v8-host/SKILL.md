---
name: ano-v8-host
description: Implement or review an ANO V8 anonymous-host UART protocol on embedded firmware. Use for 0xAB framed communication, command and parameter discovery, telemetry, UTF-8 Chinese metadata, UART ISR integration, or persisted host baud-rate configuration.
---

# ANO V8 Host

## Overview

Implement the ANO V8 host interface as a bounded asynchronous UART protocol. Keep framing, parsing, metadata discovery, control validation, and persistent configuration explicit and independently verifiable.

## Frame Contract

Use the following wire layout for every transmitted and received frame:

```text
0xAB | source | destination | id | length_lo | length_hi | payload[length] | sum | add
```

- Encode all multi-byte integer fields little-endian.
- Calculate `sum` as the wrapping sum of all bytes from `0xAB` through the payload.
- Calculate `add` as the wrapping cumulative sum over that same byte sequence.
- Reject frames whose payload length exceeds the fixed receive-buffer capacity before accepting payload bytes.
- Accept a frame only after both checksums pass and its destination is the local address or broadcast address.
- Do not use packed C structs directly as a wire format. Serialize fields with explicit byte helpers; use `memcpy` for `float` and signed 32-bit values.

## UART Architecture

Keep interrupt handlers short and defer protocol work to the main loop.

- In the RX-complete callback, push the byte into a lock-free single-producer/single-consumer ring buffer, then immediately arm the next one-byte receive.
- In the main-loop process function, drain the RX ring and advance a byte-state parser.
- Queue complete TX frames. Start the next interrupt-driven transmission only when the previous one completes.
- Protect TX queue index/count updates from ISR concurrency. Do not build or transmit a response directly inside an ISR.
- Re-arm reception after UART errors and clear local TX-active state so the protocol can recover.
- Size every TX slot for header, maximum payload, and two checksum bytes. Drop a new frame when the queue is full; never overwrite an in-flight frame.

## Discovery And Metadata

Implement discovery as data-driven tables rather than duplicated switch branches.

- Respond to parameter-discovery subcommands with device information, parameter count, current value, and parameter description.
- Emit parameter identifiers as stable `uint16_t` values. Do not reorder existing IDs: upper-computer projects may store them.
- For command discovery, first return the command count, then return the record selected by the requested command index.
- Keep each command descriptor as a fixed command byte sequence plus display name and description.
- Use the protocol's fixed-length name field exactly as specified, clearing it before copying text. Text longer than the field must be safely truncated without writing beyond it.
- UTF-8 Chinese names are byte strings: limit by byte capacity, not by displayed character count. Keep identifiers short enough that truncation does not make adjacent parameters indistinguishable.
- Follow fixed-width fields with a NUL-terminated variable-length UTF-8 description only when the receiving protocol expects that layout.

Typical parameter metadata payload:

```text
parameter_id_le16 | type | name[20] | description_utf8_nul
```

Typical command metadata payload:

```text
command_index_le16 | command_bytes[3] | reserved[8] | name[20] | description_utf8_nul
```

Validate the exact message IDs and subcommand payload shapes against the intended ANO V8 upper-computer version before implementation. The discovery concepts are stable; numeric IDs can vary by protocol release or vendor fork.

## Control Boundaries

Treat host input as untrusted control input.

- Expose application-level operations, not peripheral register writes or raw driver frames.
- Validate payload length before every field read, then validate finite floating-point values, ranges, enum values, and integer-only settings.
- Route accepted values through the application's existing public control API so its limits and side effects remain authoritative.
- Apply related PID values atomically from the application's perspective: preserve the prior value and roll it back if the driver rejects the complete update.
- Keep parameter writes separate from commands. A parameter write updates a typed value; a command performs an action and should receive the protocol acknowledgement/check frame where applicable.
- Make stop commands idempotent and safe for repeated delivery.
- Bound telemetry period and payload size. Build one coherent snapshot per frame instead of exposing transient ISR-owned state directly.

## Persisted Baud Rate

Use persistence only when the product needs the host-selected UART rate to survive reset.

1. Restrict accepted values to a product-defined whitelist, such as `115200`, `230400`, `460800`, and `921600`.
2. Store a record containing a magic value, format version, baud rate, and checksum in EEPROM or another nonvolatile store.
3. On startup, load the record only when magic, version, checksum, and whitelist validation all pass; otherwise use the compiled default.
4. After accepting a new rate, persist it first, schedule the switch after a short delay, and wait until the TX queue is empty before reinitializing UART.
5. Reapply UART-specific FIFO configuration after `HAL_UART_DeInit`/`HAL_UART_Init`, then re-arm RX interrupt reception.

Do not switch baud rate while a response is still being transmitted. Otherwise the upper computer can receive a partial frame and lose the only confirmation of the new setting.

## Nonvolatile Storage And Diagnostics

Use a verified nonvolatile backend for all host settings that must survive reset. On the STM32H750 board, a W25Qxx connected through software SPI was used when the I2C AT24C02 path was unreliable and SPI1 was already reserved for the display.

- Before trusting a serial-flash backend, issue `0x9F` and verify a plausible JEDEC ID. Treat a missing or unexpected manufacturer byte as a hardware/link failure, not as an empty configuration.
- Store coupled settings such as the parameter set and host baud rate in one versioned record: magic, format version, payload, and checksum. Load it only when every validation passes; otherwise retain compiled defaults.
- For a save command, erase the target sector, write page-aligned chunks, read the complete record back, and compare it before sending the protocol success acknowledgement. Report failure instead of accepting a save that cannot be restored after reset.
- Keep storage operations out of UART interrupts. The foreground protocol path may block for the bounded flash erase/program timeout, but RX handling must remain lightweight.
- Provide a periodic diagnostic frame when board access is limited. Include backend-ready state, current storage operation, last error/status, and JEDEC ID so a host can distinguish wiring, erase, program, and verification failures without a debugger.
- A bit-banged W25Qxx bus can avoid a shared hardware-SPI display bus. Define explicit CS, CLK, DI, and DO ownership, drive CS inactive and CLK low before probing, and check that no selected pin has a conflicting board function.

Retain an EEPROM driver without repairing it when it is no longer the active persistence backend and has no separate product requirement. Revisit EEPROM repair only when the product requires it or another I2C peripheral exposes a shared-bus fault. First verify power, address pins, and real external SDA/SCL pull-ups with a scope or logic analyzer; MCU internal pull-ups alone are normally not a reliable I2C bus design.

## Integration Checklist

- Initialize the UART peripheral before host-protocol initialization.
- Initialize the nonvolatile-storage bus before loading a persisted baud rate.
- Call the host process function from the foreground loop at a rate sufficient to drain the RX queue and meet telemetry timing.
- Forward `HAL_UART_RxCpltCallback`, `HAL_UART_TxCpltCallback`, and `HAL_UART_ErrorCallback` for the selected UART instance to the host module before other UART consumers handle that instance.
- Define one ownership boundary per UART. Do not let two protocol modules independently re-arm receive interrupts on the same handle.
- Verify with captured frames: valid discovery request, malformed length, invalid `sum`, invalid `add`, broadcast destination, maximum payload, queue-full behavior, parameter rejection, stop command acknowledgement, and delayed baud switch.

## Resources (optional)

Create only the resource directories this skill actually needs. Delete this section if no resources are required.

### scripts/
Executable code (Python/Bash/etc.) that can be run directly to perform specific operations.

**Examples from other skills:**
- PDF skill: `fill_fillable_fields.py`, `extract_form_field_info.py` - utilities for PDF manipulation
- DOCX skill: `document.py`, `utilities.py` - Python modules for document processing

**Appropriate for:** Python scripts, shell scripts, or any executable code that performs automation, data processing, or specific operations.

**Note:** Scripts may be executed without loading into context, but can still be read by Codex for patching or environment adjustments.

### references/
Documentation and reference material intended to be loaded into context to inform Codex's process and thinking.

**Examples from other skills:**
- Product management: `communication.md`, `context_building.md` - detailed workflow guides
- BigQuery: API reference documentation and query examples
- Finance: Schema documentation, company policies

**Appropriate for:** In-depth documentation, API references, database schemas, comprehensive guides, or any detailed information that Codex should reference while working.

### assets/
Files not intended to be loaded into context, but rather used within the output Codex produces.

**Examples from other skills:**
- Brand styling: PowerPoint template files (.pptx), logo files
- Frontend builder: HTML/React boilerplate project directories
- Typography: Font files (.ttf, .woff2)

**Appropriate for:** Templates, boilerplate code, document templates, images, icons, fonts, or any files meant to be copied or used in the final output.

---

**Not every skill requires all three types of resources.**
