# gs_usb_stress: host-stall reproducer for candle-usb/candleLight_fw#322

A user-space gs_usb host (Rust, [`nusb`](https://crates.io/crates/nusb), no kernel driver) that
keeps a candleLight adapter busy in both directions and repeatedly stops reading bulk IN, while
other nodes keep sending on the bus. On stock firmware this ends with an adapter that no longer
answers any USB request (including control requests on EP0) until it is power-cycled.

## What it does

1. Opens the adapter (default `1d50:606f`), claims interface 0 (on Linux this detaches the
   kernel `gs_usb` driver), sends `HOST_FORMAT`, reads `DEVICE_CONFIG` and `BT_CONST`, resets the
   channel, sets the bit timing (default 1 Mbit/s, 87.5 %), and starts it, with hardware
   timestamps if the device has them.
2. A reader thread keeps `--in-queue` (16) bulk-IN transfers queued and counts echoes and
   received frames.
3. The main thread sends standard-id frames (`0x7ff`, 8 bytes) at `--rate` (4000/s), with at
   most `--max-in-flight` (10, like the kernel driver's TX URBs) not yet echoed.
4. Every `--every` (2) s a child shell stops this process with `SIGSTOP` for the next
   `--stall-ms` value (30, 80, 120, 200, 300, 50, 250 ms, in turn) and then sends `SIGCONT`.
   While the process is stopped, at most 16 bulk-IN transfers can complete, so the adapter's
   received frames pile up in its frame pool. After each stall it sends `--burst` (80) frames at once.
5. Once a second, and after each stall: if frames are in flight and nothing came in on bulk IN
   for 300 ms, it sends `DEVICE_CONFIG`. If that fails, the adapter has hung. The tool then waits
   5 s to see whether the device comes back by itself, for example after a watchdog reset.

Output is one JSON line per second, one per stall and one at the end. Exit code 0 means the
adapter survived, 2 means it hung.

```sh
cargo run --release -- [--serial <serial>] [--seconds 600]
cargo run --release -- --help
```

## Bus

The receive load has to come from **other nodes** on the bus, so that frames arrive while the
host is stopped. In our runs eight nodes sent about 1100 frames/s in total, at 1 Mbit/s, with
29-bit ids and 8 data bytes. They ignored the `0x7ff` frames this tool sends. A second
interface running for example `cangen can0 -g 1 -e -L 8` should be a similar load (not tried).

## Results (FYSETC UCAN, STM32F072CB, macOS host)

| Firmware | Profile | Result |
|---|---|---|
| stock `85e5b33` | 2000 frames/s, stalls 80/200 ms every 5 s, burst 40, 180 s | survived, 34 stalls |
| stock `85e5b33` | defaults (as above) | **hung after 131.9 s, at the 60th stall** (a 200 ms stall followed by the burst). `DEVICE_CONFIG` then failed with `kIOReturnNotResponding` and the adapter needed a replug |
| #322 | defaults | survived 600 s and 274 stalls without a reset. 2 246 036 frames were sent and all were echoed; 5624 received CAN frames were dropped by the reserve |

The hang is probabilistic: the milder profile did not trigger it in 180 s. In the field, our
application hung the stock adapter four times in three days, each time after the host process
stalled for 80–300 ms.

**Note:** the runs above used our in-house tool, which is built on a larger gs_usb transport
that we cannot publish. This crate is a self-contained port of that tool, with the same
protocol, queue depths, in-flight limit, stall pattern and liveness check. **This crate itself
has not yet been run against hardware**, and it has only been built on macOS (nusb 0.2.9).

## Linux

With the kernel `gs_usb` driver, RX URBs are resubmitted from the completion handler, so
stopping a user-space process does not stop bulk-IN reads, and this trigger needs a stalled
kernel or USB host instead. This tool takes the interface from the kernel driver, so the same
stall pattern can be run on Linux (with permission to open the device). We have not tried it.
