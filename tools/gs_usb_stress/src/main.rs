//! Host-stall stress test for a gs_usb (candleLight_fw) adapter, from user space.
//!
//! It drives the adapter directly over USB with `nusb` (no kernel gs_usb driver), so that the
//! host's bulk-IN reading can be stopped: every `--every` seconds the process stops itself with
//! SIGSTOP for the next `--stall-ms` value, then sends a burst of `--burst` frames. Meanwhile
//! other nodes on the bus keep sending, so received frames pile up in the adapter.
//!
//! The adapter has hung if it stops answering control requests (DEVICE_CONFIG). Exit code: 0
//! survived, 2 hung or lost, 64 bad arguments.
//!
//! See README.md for the bus setup and the results on a FYSETC UCAN.

use std::process::{Command, ExitCode};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, ControlIn, ControlOut, ControlType, Direction, In, Out, Recipient};
use nusb::{Endpoint, Interface, MaybeFuture};

// gs_usb control requests (Linux drivers/net/can/usb/gs_usb.c).
const BREQ_HOST_FORMAT: u8 = 0;
const BREQ_BITTIMING: u8 = 1;
const BREQ_MODE: u8 = 2;
const BREQ_BT_CONST: u8 = 4;
const BREQ_DEVICE_CONFIG: u8 = 5;

const MODE_RESET: u32 = 0;
const MODE_START: u32 = 1;
const FLAG_LISTEN_ONLY: u32 = 1 << 0;
const FLAG_LOOP_BACK: u32 = 1 << 1;
const FLAG_HW_TIMESTAMP: u32 = 1 << 4;

const ECHO_ID_RX: u32 = 0xFFFF_FFFF;
const CAN_ERR_FLAG: u32 = 0x2000_0000;

const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);

struct Args {
    vid: u16,
    pid: u16,
    serial: Option<String>,
    bitrate: u32,
    can_id: u32,
    seconds: f64,
    rate: f64,
    stall_ms: Vec<u64>,
    every: f64,
    burst: usize,
    in_queue: usize,
    max_in_flight: usize,
    loopback: bool,
    timestamps: bool,
}

const USAGE: &str = "\
gs_usb_stress [options]
  --device VID:PID     default 1d50:606f (candleLight_fw)
  --serial S           pick one adapter by serial number
  --bitrate N          default 1000000 (needs a 48 MHz CAN clock or another exact divisor)
  --id N               standard CAN id of the frames sent, default 0x7ff
  --seconds N          default 600
  --rate N             frames/s offered, default 4000 (limited by --max-in-flight)
  --stall-ms A,B,...   stall lengths, used in turn, default 30,80,120,200,300,50,250
  --every N            seconds between stalls, default 2
  --burst N            frames sent straight after each stall, default 80
  --in-queue N         bulk-IN transfers kept queued, default 16
  --max-in-flight N    frames sent but not yet echoed, default 10 (Linux gs_usb's TX URBs)
  --loopback           LISTEN_ONLY | LOOP_BACK: nothing goes on the bus (not tested)
  --no-timestamps      do not ask for hardware timestamps";

fn parse() -> Result<Args, String> {
    let mut a = Args {
        vid: 0x1d50,
        pid: 0x606f,
        serial: None,
        bitrate: 1_000_000,
        can_id: 0x7ff,
        seconds: 600.0,
        rate: 4000.0,
        stall_ms: vec![30, 80, 120, 200, 300, 50, 250],
        every: 2.0,
        burst: 80,
        in_queue: 16,
        max_in_flight: 10,
        loopback: false,
        timestamps: true,
    };
    let num = |k: &str, v: &str| -> Result<u64, String> {
        let r = match v.strip_prefix("0x") {
            Some(h) => u64::from_str_radix(h, 16),
            None => v.parse(),
        };
        r.map_err(|e| format!("{k}: {e}"))
    };
    let mut it = std::env::args().skip(1);
    while let Some(k) = it.next() {
        let mut v = || it.next().ok_or_else(|| format!("{k} needs a value"));
        match k.as_str() {
            "--device" => {
                let s = v()?;
                let (vid, pid) = s.split_once(':').ok_or("--device: VID:PID")?;
                a.vid = u16::from_str_radix(vid, 16).map_err(|e| format!("--device: {e}"))?;
                a.pid = u16::from_str_radix(pid, 16).map_err(|e| format!("--device: {e}"))?;
            }
            "--serial" => a.serial = Some(v()?),
            "--bitrate" => a.bitrate = num(&k, &v()?)? as u32,
            "--id" => a.can_id = num(&k, &v()?)? as u32 & 0x7ff,
            "--seconds" => a.seconds = v()?.parse().map_err(|e| format!("{k}: {e}"))?,
            "--rate" => a.rate = v()?.parse().map_err(|e| format!("{k}: {e}"))?,
            "--every" => a.every = v()?.parse().map_err(|e| format!("{k}: {e}"))?,
            "--burst" => a.burst = num(&k, &v()?)? as usize,
            "--in-queue" => a.in_queue = (num(&k, &v()?)? as usize).max(1),
            "--max-in-flight" => a.max_in_flight = (num(&k, &v()?)? as usize).max(1),
            "--stall-ms" => {
                a.stall_ms = v()?
                    .split(',')
                    .filter(|s| !s.is_empty())
                    .map(|s| num(&k, s.trim()))
                    .collect::<Result<_, _>>()?;
            }
            "--loopback" => a.loopback = true,
            "--no-timestamps" => a.timestamps = false,
            "-h" | "--help" => return Err(USAGE.to_owned()),
            other => return Err(format!("unknown argument {other}\n{USAGE}")),
        }
    }
    Ok(a)
}

fn le32(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

struct Device {
    interface: Interface,
    bulk_out: Endpoint<Bulk, Out>,
    bulk_in: Option<Endpoint<Bulk, In>>,
}

fn open(a: &Args) -> Result<Device, String> {
    let info = nusb::list_devices()
        .wait()
        .map_err(|e| format!("listing USB devices: {e}"))?
        .find(|d| {
            d.vendor_id() == a.vid
                && d.product_id() == a.pid
                && a.serial.as_deref().is_none_or(|s| d.serial_number() == Some(s))
        })
        .ok_or_else(|| format!("no {:04x}:{:04x} device found", a.vid, a.pid))?;
    let device = info.open().wait().map_err(|e| format!("open: {e}"))?;
    let interface = device
        .detach_and_claim_interface(0)
        .wait()
        .map_err(|e| format!("claim interface 0: {e}"))?;
    let (mut in_addr, mut out_addr) = (None, None);
    if let Some(desc) = interface.descriptor() {
        for ep in desc.endpoints() {
            if ep.transfer_type() == TransferType::Bulk {
                match ep.direction() {
                    Direction::In => in_addr = in_addr.or(Some(ep.address())),
                    Direction::Out => out_addr = out_addr.or(Some(ep.address())),
                }
            }
        }
    }
    let (Some(i), Some(o)) = (in_addr, out_addr) else {
        return Err("interface 0 has no bulk IN and OUT endpoints".into());
    };
    let bulk_in = interface.endpoint::<Bulk, In>(i).map_err(|e| format!("bulk IN: {e}"))?;
    let bulk_out = interface.endpoint::<Bulk, Out>(o).map_err(|e| format!("bulk OUT: {e}"))?;
    Ok(Device { interface, bulk_out, bulk_in: Some(bulk_in) })
}

impl Device {
    fn control_out(&self, request: u8, value: u16, data: &[u8]) -> Result<(), String> {
        self.interface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Interface,
                    request,
                    value,
                    index: 0,
                    data,
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(|e| format!("control request {request}: {e}"))
    }

    fn control_in(&self, request: u8, value: u16, length: u16) -> Result<Vec<u8>, String> {
        self.interface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Interface,
                    request,
                    value,
                    index: 0,
                    length,
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(|e| format!("control request {request}: {e}"))
    }

    /// The liveness check: a hung adapter does not answer this.
    fn device_config(&self) -> Result<Vec<u8>, String> {
        let r = self.control_in(BREQ_DEVICE_CONFIG, 0, 12)?;
        if r.len() < 12 {
            return Err(format!("DEVICE_CONFIG reply of {} bytes", r.len()));
        }
        Ok(r)
    }
}

/// brp, tseg1, tseg2 for `bitrate` with a sample point near 87.5 %.
fn bit_timing(bt: &[u8], bitrate: u32) -> Result<[u32; 3], String> {
    let (fclk, t1min, t1max, t2min, t2max) = (le32(bt, 4), le32(bt, 8), le32(bt, 12), le32(bt, 16), le32(bt, 20));
    let (brp_min, brp_max, brp_inc) = (le32(bt, 28), le32(bt, 32), le32(bt, 36).max(1));
    let mut best: Option<(u32, [u32; 3])> = None;
    let mut brp = brp_min.max(1);
    while brp <= brp_max {
        let div = u64::from(brp) * u64::from(bitrate);
        if div != 0 && u64::from(fclk) % div == 0 {
            let q = (u64::from(fclk) / div) as u32;
            if q >= 1 + t1min + t2min && q <= 1 + t1max + t2max {
                let t2 = ((q * 125 + 500) / 1000).clamp(t2min, t2max).max(q.saturating_sub(1 + t1max));
                let t1 = q - 1 - t2;
                let err = ((1 + t1) * 1000 / q).abs_diff(875);
                if t1 >= t1min && t1 <= t1max && best.is_none_or(|(e, _)| err < e) {
                    best = Some((err, [brp, t1, t2]));
                }
            }
        }
        brp += brp_inc;
    }
    best.map(|(_, t)| t).ok_or_else(|| format!("no bit timing for {bitrate} bit/s at {fclk} Hz"))
}

#[derive(Default)]
struct Shared {
    slots: Mutex<Vec<bool>>,
    freed: Condvar,
    stop: AtomicBool,
    last_in_ms: AtomicU64,
    echoes: AtomicU64,
    rx: AtomicU64,
    err_frames: AtomicU64,
    overflow_flags: AtomicU64,
    bad: AtomicU64,
    in_error: Mutex<Option<String>>,
}

fn reader(mut ep: Endpoint<Bulk, In>, queue: usize, s: &Shared, t0: Instant) {
    let size = ep.max_packet_size().max(1);
    while !s.stop.load(Ordering::Acquire) {
        while ep.pending() < queue {
            let b = ep.allocate(size);
            ep.submit(b);
        }
        let Some(c) = ep.wait_next_complete(Duration::from_millis(100)) else { continue };
        if let Err(e) = c.status {
            *s.in_error.lock().unwrap() = Some(format!("bulk IN: {e}"));
            return;
        }
        let b = &c.buffer[..c.actual_len.min(c.buffer.len())];
        s.last_in_ms.store(t0.elapsed().as_millis() as u64, Ordering::Release);
        if b.len() < 20 {
            s.bad.fetch_add(1, Ordering::Relaxed);
            continue;
        }
        let echo_id = le32(b, 0);
        if echo_id == ECHO_ID_RX {
            if le32(b, 4) & CAN_ERR_FLAG != 0 {
                s.err_frames.fetch_add(1, Ordering::Relaxed);
            } else {
                s.rx.fetch_add(1, Ordering::Relaxed);
            }
            if b[10] & 1 != 0 {
                s.overflow_flags.fetch_add(1, Ordering::Relaxed);
            }
            continue;
        }
        let mut slots = s.slots.lock().unwrap();
        match slots.get_mut(echo_id as usize) {
            Some(slot @ true) => {
                *slot = false;
                s.echoes.fetch_add(1, Ordering::Relaxed);
                drop(slots);
                s.freed.notify_all();
            }
            _ => {
                s.bad.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
    ep.cancel_all();
    while ep.pending() > 0 && ep.wait_next_complete(Duration::from_millis(200)).is_some() {}
}

struct Sender {
    counter: u32,
    sent: u64,
    write_timeouts: u64,
}

enum Fail {
    Hung(String),
}

impl Sender {
    /// Sends one frame; waits up to 100 ms for a free echo slot. Ok(false) on a timeout.
    fn send(&mut self, dev: &mut Device, s: &Shared, a: &Args, ts: bool) -> Result<bool, String> {
        let deadline = Instant::now() + Duration::from_millis(100);
        let slot = {
            let mut slots = s.slots.lock().unwrap();
            loop {
                if let Some(i) = slots.iter().position(|b| !*b) {
                    slots[i] = true;
                    break Some(i);
                }
                let left = deadline.saturating_duration_since(Instant::now());
                if left.is_zero() {
                    break None;
                }
                slots = s.freed.wait_timeout(slots, left).unwrap().0;
            }
        };
        let Some(slot) = slot else {
            self.write_timeouts += 1;
            return Ok(false);
        };
        while dev.bulk_out.pending() > 0 {
            match dev.bulk_out.wait_next_complete(Duration::ZERO) {
                Some(c) => c.status.map_err(|e| format!("bulk OUT: {e}"))?,
                None => break,
            }
        }
        if dev.bulk_out.pending() >= 32 {
            match dev.bulk_out.wait_next_complete(Duration::from_millis(100)) {
                Some(c) => c.status.map_err(|e| format!("bulk OUT: {e}"))?,
                None => {
                    s.slots.lock().unwrap()[slot] = false;
                    self.write_timeouts += 1;
                    return Ok(false);
                }
            }
        }
        self.counter = self.counter.wrapping_add(1);
        let mut f = Vec::with_capacity(24);
        f.extend_from_slice(&(slot as u32).to_le_bytes());
        f.extend_from_slice(&a.can_id.to_le_bytes());
        f.extend_from_slice(&[8, 0, 0, 0]);
        f.extend_from_slice(&self.counter.to_le_bytes());
        f.extend_from_slice(&[0; 4]);
        if ts {
            f.extend_from_slice(&[0; 4]);
        }
        dev.bulk_out.submit(Buffer::from(f));
        self.sent += 1;
        Ok(true)
    }
}

fn main() -> ExitCode {
    let a = match parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(64);
        }
    };
    match run(&a) {
        Ok(()) => ExitCode::SUCCESS,
        Err(Fail::Hung(e)) => {
            println!("{{\"event\":\"failed\",\"error\":{e:?}}}");
            // Did it come back by itself (e.g. a watchdog reset), or is a replug needed?
            let t = Instant::now();
            while t.elapsed() < Duration::from_secs(5) {
                if let Ok(d) = open(&a) {
                    if d.device_config().is_ok() {
                        println!(
                            "{{\"event\":\"recovered\",\"after_s\":{:.2}}}",
                            t.elapsed().as_secs_f64()
                        );
                        return ExitCode::from(2);
                    }
                }
                thread::sleep(Duration::from_millis(100));
            }
            println!("{{\"event\":\"not_answering\",\"detail\":\"no DEVICE_CONFIG answer for 5 s: replug needed\"}}");
            ExitCode::from(2)
        }
    }
}

fn run(a: &Args) -> Result<(), Fail> {
    let hung = Fail::Hung;
    let mut dev = open(a).map_err(hung)?;
    dev.control_out(BREQ_HOST_FORMAT, 1, &0x0000_beef_u32.to_le_bytes()).map_err(hung)?;
    let config = dev.device_config().map_err(hung)?;
    let bt = dev.control_in(BREQ_BT_CONST, 0, 40).map_err(hung)?;
    if bt.len() < 40 {
        return Err(hung(format!("BT_CONST reply of {} bytes", bt.len())));
    }
    let [brp, tseg1, tseg2] = bit_timing(&bt, a.bitrate).map_err(hung)?;
    let ts = a.timestamps && le32(&bt, 0) & FLAG_HW_TIMESTAMP != 0;
    let mode = |m: u32, f: u32| [m.to_le_bytes(), f.to_le_bytes()].concat();
    dev.control_out(BREQ_MODE, 0, &mode(MODE_RESET, 0)).map_err(hung)?;
    let prop = tseg1 / 2;
    let timing: Vec<u8> = [prop, tseg1 - prop, tseg2, 1, brp].iter().flat_map(|v| v.to_le_bytes()).collect();
    dev.control_out(BREQ_BITTIMING, 0, &timing).map_err(hung)?;

    let t0 = Instant::now();
    let s = Arc::new(Shared {
        slots: Mutex::new(vec![false; a.max_in_flight]),
        ..Shared::default()
    });
    let ep = dev.bulk_in.take().expect("bulk IN");
    let (rs, queue) = (Arc::clone(&s), a.in_queue);
    let reader = thread::spawn(move || reader(ep, queue, &rs, t0));

    let flags = if a.loopback { FLAG_LISTEN_ONLY | FLAG_LOOP_BACK } else { 0 } | if ts { FLAG_HW_TIMESTAMP } else { 0 };
    dev.control_out(BREQ_MODE, 0, &mode(MODE_START, flags)).map_err(hung)?;
    println!(
        "{{\"event\":\"start\",\"device\":\"{:04x}:{:04x}\",\"sw_version\":{},\"hw_version\":{},\"fclk\":{},\"brp\":{brp},\"tseg1\":{tseg1},\"tseg2\":{tseg2},\"timestamps\":{ts},\"seconds\":{},\"rate\":{},\"stall_ms\":{:?},\"every\":{},\"burst\":{},\"in_queue\":{},\"max_in_flight\":{},\"loopback\":{}}}",
        a.vid, a.pid, le32(&config, 4), le32(&config, 8), le32(&bt, 4),
        a.seconds, a.rate, a.stall_ms, a.every, a.burst, a.in_queue, a.max_in_flight, a.loopback
    );

    let result = stress(a, &mut dev, &s, ts, t0);
    s.stop.store(true, Ordering::Release);
    let _ = reader.join();
    if result.is_ok() {
        let _ = dev.control_out(BREQ_MODE, 0, &mode(MODE_RESET, 0));
    }
    result
}

fn stress(a: &Args, dev: &mut Device, s: &Shared, ts: bool, t0: Instant) -> Result<(), Fail> {
    let pid = std::process::id();
    let period = Duration::from_secs_f64(1.0 / a.rate.max(1.0));
    let mut tx = Sender { counter: 0, sent: 0, write_timeouts: 0 };
    let mut next_send = Instant::now();
    let mut next_report = t0 + Duration::from_secs(1);
    let mut next_stall = t0 + Duration::from_secs_f64(a.every);
    let mut stalls = 0usize;

    // A hang shows as: frames in flight, nothing on bulk IN for 300 ms, and DEVICE_CONFIG fails.
    let check = |dev: &Device, s: &Shared| -> Result<(), Fail> {
        if let Some(e) = s.in_error.lock().unwrap().clone() {
            return Err(Fail::Hung(e));
        }
        let in_flight = s.slots.lock().unwrap().iter().any(|b| *b);
        let quiet = t0.elapsed().as_millis() as u64 - s.last_in_ms.load(Ordering::Acquire);
        if in_flight && quiet > 300 {
            dev.device_config()
                .map_err(|e| Fail::Hung(format!("nothing received for {quiet} ms with frames in flight, and {e}")))?;
        }
        Ok(())
    };

    while t0.elapsed().as_secs_f64() < a.seconds {
        if Instant::now() >= next_stall && !a.stall_ms.is_empty() {
            let ms = a.stall_ms[stalls % a.stall_ms.len()];
            stalls += 1;
            let script = format!("sleep 0.005; kill -STOP {pid}; sleep {}; kill -CONT {pid}", ms as f64 / 1000.0);
            let started = Instant::now();
            Command::new("sh").arg("-c").arg(&script).spawn().map_err(|e| Fail::Hung(format!("spawn sh: {e}")))?;
            // Keep sending until the stop has landed and lasted; the longest gap measures it.
            let (mut longest, mut tick) = (Duration::ZERO, Instant::now());
            while started.elapsed() < Duration::from_millis(ms + 300) && longest < Duration::from_millis(ms) {
                tx.send(dev, s, a, ts).map_err(Fail::Hung)?;
                let n = Instant::now();
                longest = longest.max(n - tick);
                tick = n;
            }
            let mut burst_timeouts = 0;
            for _ in 0..a.burst {
                if !tx.send(dev, s, a, ts).map_err(Fail::Hung)? {
                    burst_timeouts += 1;
                }
            }
            println!(
                "{{\"event\":\"stall\",\"n\":{stalls},\"t\":{:.3},\"asked_ms\":{ms},\"measured_ms\":{:.1},\"burst_timeouts\":{burst_timeouts}}}",
                t0.elapsed().as_secs_f64(),
                longest.as_secs_f64() * 1000.0
            );
            check(dev, s)?;
            next_stall = Instant::now() + Duration::from_secs_f64(a.every);
            next_send = Instant::now();
        }
        if Instant::now() >= next_send {
            tx.send(dev, s, a, ts).map_err(Fail::Hung)?;
            next_send += period;
            if next_send + Duration::from_millis(50) < Instant::now() {
                next_send = Instant::now();
            }
        }
        if Instant::now() >= next_report {
            check(dev, s)?;
            println!(
                "{{\"event\":\"stats\",\"t\":{:.1},\"sent\":{},\"echoes\":{},\"rx\":{},\"error_frames\":{},\"overflow_flags\":{},\"write_timeouts\":{},\"stalls\":{stalls}}}",
                t0.elapsed().as_secs_f64(),
                tx.sent,
                s.echoes.load(Ordering::Relaxed),
                s.rx.load(Ordering::Relaxed),
                s.err_frames.load(Ordering::Relaxed),
                s.overflow_flags.load(Ordering::Relaxed),
                tx.write_timeouts
            );
            next_report += Duration::from_secs(1);
        }
        let wait = next_send.saturating_duration_since(Instant::now()).min(Duration::from_millis(2));
        if !wait.is_zero() {
            thread::sleep(wait);
        }
    }
    check(dev, s)?;
    println!(
        "{{\"event\":\"survived\",\"seconds\":{:.1},\"stalls\":{stalls},\"sent\":{},\"echoes\":{},\"rx\":{}}}",
        t0.elapsed().as_secs_f64(),
        tx.sent,
        s.echoes.load(Ordering::Relaxed),
        s.rx.load(Ordering::Relaxed)
    );
    Ok(())
}
