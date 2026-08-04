# CANnectivity throughput analysis — STM32G474, gs_usb/SocketCAN

Analysis of the CANnectivity data path with respect to replacing a modified `candleLight_fw`
(FD) build on our USB-to-CAN adapters. Target silicon: **STM32G474** — Cortex-M4F @ 170 MHz,
FDCAN (Bosch M_CAN), **USB full-speed device controller** (PMA-based, non-OTG; no high speed).

Tree analysed: `e497cdd` (`v1.4.0-dev`, Nov 2025). All `file:line` references are against that
commit. Upstream claims were re-checked against `CANnectivity/cannectivity@main` while writing.

---

## 1. Verdict

**As shipped, CANnectivity does not match candleLight-FD, and the gap is large.** The causes are
implementation details, not architecture — every one of them is identified in §3 and §4, and none
requires redesigning the firmware.

**After the Tier-0 and Tier-1 items in §5, it should reach the same ceiling.** That ceiling is a
property of the *gs_usb protocol over USB full speed*, and it binds both firmwares equally:

| Constraint | Consequence |
|---|---|
| Linux `gs_usb` parses exactly **one host frame per URB** (`gs_usb_receive_bulk_callback()` takes `hf = urb->transfer_buffer` and handles one frame; RX URBs are `hf_size_rx` bytes) | Packing several CAN frames into one bulk transfer is impossible without breaking the mainline driver. Frame rate is transaction-rate bound, not bandwidth bound. |
| A CAN FD host frame is a **fixed 80 bytes** (12 B header + 64 B payload + 4 B timestamp) regardless of DLC — see `gs_usb_can_rx_callback()` copying `can_dlc_to_bytes(CANFD_MAX_DLC)` unconditionally at `subsys/usb/device_next/class/gs_usb.c:1081-1085` | ~48% wire efficiency on FD. Protocol-level; unfixable inside gs_usb. |
| STM32G474 has no USB HS controller | 12 Mbit/s FS is the hard link limit. |

So "exceed candleLight-FD" realistically means *saturate the same link at lower CPU cost with
better correctness*, not *go several times faster*. Both firmwares converge on the same wall.

What CANnectivity is worth adopting for, once fixed:

- It is Zephyr, like the rest of our stack, and already in our west manifest.
- 170 MHz M4F versus candleLightFD's 64 MHz STM32G0B1 — real headroom for FD at high data rates.
- **TX echo frames are timestamped at actual transmission completion**
  (`gs_usb_can_tx_callback()`, `subsys/usb/device_next/class/gs_usb.c:1160-1207`, invoked from the
  M_CAN TX event FIFO handler). candleLight timestamps at hand-off to the peripheral, which
  corrupts TX timestamps and one-shot semantics —
  [candleLight_fw#175](https://github.com/candle-usb/candleLight_fw/issues/175). This is a
  correctness advantage we would gain, not lose.

**Before any of this: rebase.** This tree is Nov 2025. Upstream shipped **v1.4.0 in Apr 2026**,
which made `device_next` the default stack and deprecated the legacy class, and landed several
`net_buf` lifetime and leak fixes in Jul 2026 that matter specifically under load. The two
structural bottlenecks below were verified still present on upstream `main` at time of writing,
so rebasing does not make this document obsolete — it just removes noise.

---

## 2. The data path as it stands

There are two complete gs_usb class implementations, selected by USB stack:

| Stack | File | Selected by |
|---|---|---|
| `device_next` (UDC) | `subsys/usb/device_next/class/gs_usb.c` | `CONFIG_USB_DEVICE_STACK_NEXT=y` → `prj_usbd_next*.conf` |
| legacy (deprecated) | `subsys/usb/device/class/gs_usb.c` | `CONFIG_USB_DEVICE_STACK=y` → **`prj.conf`, the default in this tree** |

Only `device_next` is worth discussing for our purposes (see §3.1). Topology, both directions:

```
device → host   CAN RX ISR ─→ net_buf alloc ─→ rx_fifo ─→ gs_usb_rx thread
                          ─→ usbd_ep_enqueue ─→ [block until completion] ─→ host

host → device   bulk OUT completion ─→ tx_fifo ─→ gs_usb_tx thread
                          ─→ can_send(K_FOREVER) ─→ TX-done cb ─→ echo re-injected into rx_fifo
```

Notable properties:

- **The application layer is not on the data path at all.** `app/src/main.c` registers channels
  and three callbacks (timestamp, LED event, termination) and never touches a frame. There is no
  app-level msgq, workqueue or copy. All performance behaviour lives in the class driver.
- **One shared `net_buf` pool** for RX, TX and every channel:
  `UDC_BUF_POOL_DEFINE(gs_usb_pool_##n, CONFIG_USBD_GS_USB_POOL_SIZE, GS_USB_HOST_FRAME_MAX_SIZE, ...)`
  at `subsys/usb/device_next/class/gs_usb.c:1768`, default **20** buffers (21 with compatibility
  mode). With `CONFIG_CAN_FD_MODE=y` and timestamps, `GS_USB_HOST_FRAME_MAX_SIZE` is 80 bytes
  (`include/cannectivity/usb/class/gs_usb.h:494-500`) — so the entire pipeline is ~1.6 KB of buffer.
- **TX echo frames share the IN pipe with real RX frames.** Every host TX costs one extra IN
  transfer, so a busy TX stream halves effective RX capacity.
- Two `k_fifo`s, unbounded; the pool is the only real bound.
- Two threads, `gs_usb_rx` and `gs_usb_tx`, both 1024 B stack, both **priority 0**
  (`subsys/usb/device_next/class/Kconfig.gs_usb`), created in `gs_usb_preinit()` at `:1603-1611`.
- Exactly two accept-all CAN RX filters per channel (std + ext, id 0 / mask 0),
  `gs_usb_register_channel()` at `:1496-1531`.

---

## 3. Why it is slow

### 3.1 The default build is instrumented, on the deprecated stack

This is almost certainly what was measured in the earlier evaluation, and it dominates everything
else.

`app/prj.conf` — the config you get from a plain `west build app/` — contains:

```
CONFIG_USB_DEVICE_STACK=y                        # deprecated stack
CONFIG_USB_DEVICE_GS_USB_LOG_LEVEL_DBG=y         # debug logging
CONFIG_CANNECTIVITY_LOG_LEVEL_DBG=y
```

and both hot paths hexdump every single frame:

- `subsys/usb/device_next/class/gs_usb.c:1133` — `LOG_HEXDUMP_DBG(buf->data, buf->len, "RX host frame");`
- `subsys/usb/device_next/class/gs_usb.c:1230` — `LOG_HEXDUMP_DBG(buf->data, buf->len, "TX host frame");`
- legacy equivalents at `subsys/usb/device/class/gs_usb.c:999` and `:1140`

A benchmark on that build measures the logging subsystem, not the CAN path.

The legacy stack is additionally slower by construction:

- IN transfers are **synchronous and blocking**:
  `usb_transfer_sync(ep, buf->data, buf->len, USB_TRANS_WRITE)` at
  `subsys/usb/device/class/gs_usb.c:1001`.
- OUT transfers bounce through static per-endpoint buffers and are copied a second time into a
  pool buffer — `net_buf_add_mem(buf, data->tx_buffer2, tsize)` at `:1090`.

→ **Build `-DFILE_SUFFIX=usbd_next_release`.** `app/prj_usbd_next_release.conf` selects
`device_next` and omits `CONFIG_LOG` entirely.

### 3.2 Exactly one bulk IN transfer in flight, ever

This is the structural throughput limit, and it is the single most valuable thing to change.

`subsys/usb/device_next/class/gs_usb.c:1599`:

```c
k_sem_init(&data->in_sem, 0, 1);
```

`gs_usb_rx_thread()`, `:1135-1143`:

```c
err = usbd_ep_enqueue(config->c_data, buf);
if (err != 0) { ... continue; }

k_sem_take(&data->in_sem, K_FOREVER);   /* released only in gs_usb_request() on IN completion */
```

The semaphore is given only from the completion handler (`gs_usb_request()`, `:1344-1348`). So
every device→host frame costs:

```
CAN RX ISR → k_fifo_put → [ctx switch] rx_thread → usbd_ep_enqueue
  → [ctx switch] UDC worker → hardware transfer → completion ISR
  → [ctx switch] UDC worker → gs_usb_request() → k_sem_give
  → [ctx switch] rx_thread → next usbd_ep_enqueue
```

Two full thread round-trips per frame, with the bulk IN endpoint sitting **NAK'ed** for the whole
turnaround. Meanwhile the host is not the constraint at all: the Linux driver defines
`GS_MAX_RX_URBS 30` and keeps thirty RX URBs submitted, waiting for data that the device is
refusing to hand over.

Zephyr's `udc_stm32` already supports depth here. `udc_stm32_ep_enqueue()` does
`udc_buf_put(ep_cfg, buf)` and only starts a transfer if the endpoint is idle; the completion
path does `udc_buf_peek(ep_cfg)` and chains the next queued buffer from the UDC worker thread,
with no class-driver involvement. Queueing several IN buffers therefore removes `gs_usb_rx_thread`
from the per-frame critical path entirely.

There is also an accounting bug in the same mechanism: the `err != 0` branch of `gs_usb_request()`
(`:1310-1319`) unrefs the buffer and returns **without** `k_sem_give(&data->in_sem)`. One failed
IN transfer leaves the RX thread permanently off by one. `gs_usb_disable()` calls
`k_sem_reset()` at `:1414`, so a cable replug clears it, but an in-session endpoint error does not.
The `k_sem_take()` return value at `:1143` is also unchecked, so the `-EAGAIN` from that reset is
ignored and the thread proceeds with a stale channel index.

### 3.3 Only one bulk OUT transfer armed per endpoint

The mirror image on the host→device side. `gs_usb_request()` handles an OUT completion by queueing
the buffer and then allocating a fresh one to re-arm:

```c
k_fifo_put(&data->tx_fifo, buf);            /* :1335 */
ret = gs_usb_out_start(c_data, bi->ep);     /* :1337 */
```

`gs_usb_out_start()` (`:895-921`) allocates one pool buffer and enqueues one transfer. The host is
NAK'ed for the duration of the re-arm gap, on every single frame.

### 3.4 A 20-buffer pool, shared across everything

`CONFIG_USBD_GS_USB_POOL_SIZE` defaults to 20 (21 in compatibility mode) and the Kconfig help says
plainly: *"The pool is used for both RX and TX, and shared between all channels."*

For comparison, `candleLight_fw` uses `CAN_QUEUE_SIZE = 64 * NUM_CAN_CHANNEL` — 64 frames **per
channel**. And because a TX frame holds its pool buffer from `can_send()` all the way through to
the TX-done callback, a slow or stalled TX channel drains the pool out from under CAN RX. That
coupling is what turns §4 from a stall into a total freeze.

When the pool is empty, `gs_usb_can_rx_callback()` (`:1045-1050`) drops the frame, logs, and bumps
a per-channel `rx_overflows` semaphore; the `GS_USB_CAN_FLAG_OVERFLOW` bit is then set on the
*next* frame that does get through (`:1129-1131`). That is the only drop signal the host ever sees.

---

## 4. Why it froze under simultaneous TX and RX

Three defects compound. This is upstream issue
[#217](https://github.com/CANnectivity/cannectivity/issues/217) — **open, unfixed**, reported
Jun 2026 on an STM32G473, which is effectively our silicon.

The failure sequence:

1. A channel's CAN TX cannot complete — no ACK on the bus (nothing else connected, or the peer is
   down), so the M_CAN TX buffers never free.
2. `gs_usb_tx_thread()` calls
   `can_send(channel->dev, &frame, K_FOREVER, gs_usb_can_tx_callback, buf)` at
   `subsys/usb/device_next/class/gs_usb.c:1294`. Zephyr's `can_mcan_send()` takes `data->tx_sem`
   with that timeout, so **the thread blocks forever**.
3. There is exactly **one** `gs_usb_tx_thread` and **one** `tx_fifo` for all channels
   (`struct gs_usb_data`, `:65-89`). Head-of-line block: one dead bus wedges every channel's TX.
4. The OUT endpoint keeps completing and keeps allocating pool buffers into `tx_fifo`, which
   nobody is draining. The 20-buffer pool empties.
5. `gs_usb_out_start()` now returns `-ENOMEM` (`:908-912`). In `gs_usb_request()` this error is
   **only logged** (`:1338-1341`) — there is no retry, and `gs_usb_out_start()` is otherwise only
   ever called from `gs_usb_enable()`. **The OUT pipe is now permanently dead** until USB
   re-enable, even if the bus recovers.
6. CAN RX also starves, because it draws from the same empty pool.

RX survives in the reporter's account because the RX thread is separate and some buffers cycle,
but TX never comes back without a power cycle. This matches the freeze we hit.

For reference, Elmue's CANable 2.5 firmware added a **500 ms TX timeout** for exactly this
scenario, and candleLight has its own variant of the bug
([candleLight_fw#58](https://github.com/candle-usb/candleLight_fw/issues/58)) — so this is a
well-known class of failure, not something unique to CANnectivity.

---

## 5. Ranked optimizations

Impact estimates are engineering judgement from the code paths above, **not measurements** — §8
exists so they can be replaced with real numbers.

### Tier 0 — configuration only. Do this before measuring anything.

| # | Change | Impact | Effort | Risk |
|---|---|---|---|---|
| 0.1 | Build `-DFILE_SUFFIX=usbd_next_release` instead of the default `prj.conf` | **Very large.** Removes per-frame hexdump logging and the blocking legacy stack | None | None |
| 0.2 | `CONFIG_SPEED_OPTIMIZATIONS=y`, optionally `CONFIG_LTO=y` | Single-digit to low-teens % | One line | Flash size — irrelevant on 512 KB G474 |

On 0.2: **no `prj*.conf` in the repo sets any optimization level**, so every build is `-Os`. The
only `CONFIG_LTO=y` in the tree is in `app/boards/mks_canable_v20_stm32g431xx.conf` and it is there
to save flash, not time. This workload is per-frame-CPU-cost bound, so the default is the wrong
trade for us.

### Tier 1 — structural. These are the real wins.

| # | Change | Impact | Effort | Risk | Files |
|---|---|---|---|---|---|
| 1.1 | **Pipeline the bulk IN endpoint.** Remove `in_sem` as a one-at-a-time gate; keep 4–8 IN transfers queued, capped by an atomic in-flight counter, with the pool as back-pressure | **Largest single win.** Removes two context switches and the NAK gap per frame | Medium | Medium — buffer lifetime and `gs_usb_disable()` teardown need care | `subsys/usb/device_next/class/gs_usb.c` (`gs_usb_rx_thread`, `gs_usb_request`, `gs_usb_preinit`) |
| 1.2 | **Fix the TX head-of-line block.** Bounded `can_send()` timeout; per-channel TX handling; re-arm OUT when a buffer returns to the pool | Removes the freeze entirely | Medium | Low | same file (`gs_usb_tx_thread`, `gs_usb_request`, `gs_usb_out_start`) |
| 1.3 | **Keep 2–4 OUT transfers armed** per OUT endpoint instead of re-arming after each completion | Meaningful on host→device throughput | Small | Low | `gs_usb_out_start`, `gs_usb_enable` |
| 1.4 | **Split RX and TX pools; raise the count to 48–64** | Decouples the two directions, absorbs bursts | Small | Low — 64 × 80 B = 5 KB against 128 KB SRAM | `Kconfig.gs_usb`, `UDC_BUF_POOL_DEFINE` at `:1768` |

1.1 and 1.2 are complementary and should land together: pipelining IN without fixing the TX stall
just makes the pool drain faster.

Both 1.1 and 1.2 are upstreamable and there is no competing upstream work in flight — worth
contributing back rather than carrying as a fork delta.

### Tier 2 — STM32G474 specific, in our own board files

**2.1 — Re-partition the FDCAN message RAM.** Zephyr's `dts/arm/st/g4/stm32g4.dtsi` ships:

```dts
bosch,mram-cfg = <0x0 28 8 3 3 0 3 3>;
/*                off std ext f0 f1 rxb txe txb */
```

Three problems, all fixable from our overlay:

- **RX FIFO0 holds three elements.** At CAN FD 1M/5M arbitration/data, back-to-back frames land
  roughly every 50 µs, so the hardware absorbs about 150 µs of burst before overrunning. That is
  nothing if the USB path stalls at all — and per §3.2 it stalls on every frame today.
- **RX FIFO1's three elements are dead RAM.** Zephyr's `can_mcan.c` routes *all* filters to FIFO0
  (`filter_element.sfec = CAN_MCAN_XFEC_FIFO0`, `.efec = CAN_MCAN_XFEC_FIFO0`, with the comment
  *"Always use FIFO0 to ensure frames are delivered to callbacks in the order received"*). Nothing
  is ever placed in FIFO1. Same for the RX buffer section — it is set up but has no interrupt
  handling.
- **36 filter elements are allocated and 2 are used.** CANnectivity installs exactly one accept-all
  standard filter and one accept-all extended filter (`gs_usb_register_channel()`, `:1496-1531`).

Reclaiming FIFO1 (3 × 72 B for FD elements) and the unused filters (28 × 4 B + 8 × 8 B, less the
two we keep ≈ 164 B) frees roughly 390 bytes of the ~848 B per-instance window
(`reg = <0x40006400 0x400>, <0x4000a400 0x350>`), which is enough to take RX FIFO0 from 3 to
around 8 elements and grow the TX buffers:

```dts
&fdcan1 {
	/* 1 std filter, 1 ext filter (all CANnectivity uses), no FIFO1, no RX buffers */
	bosch,mram-cfg = <0x0 1 1 8 0 0 4 4>;
};
```

**Verify the element count against RM0440 for our exact part and FDCAN instance layout before
flashing** — the numbers above assume 72-byte FD elements and the single-instance window from the
Zephyr dtsi. If we only ever use one FDCAN instance, there may be more of the shared block
available than the dtsi's default offset assumes.

**2.2 — Thread priority discipline.** Both gs_usb threads default to priority **0**, i.e.
preemptible. The LED state machine runs on the **system workqueue** — `k_work_poll_submit()` at
`app/src/led.c:523` and `:576`, handler at `:504` — which sits at Zephyr's default *cooperative*
priority −1 and therefore preempts the CAN data path. So does the logging thread. Either raise
`CONFIG_USBD_GS_USB_{RX,TX}_THREAD_PRIO` into the cooperative range above the workqueue, or move
the LED FSM onto its own low-priority workqueue. Bump the 1024 B stacks at the same time, once IN
pipelining adds state.

**2.3 — `CONFIG_USBD_GS_USB_COMPATIBILITY_MODE=n`** if our hosts are all Linux ≥ 6.12.5. Drops a
second bulk OUT endpoint, a pool buffer and a branch per completion. Small but free. Keep it on if
anything talks to these adapters through python-can or an older kernel, which still hardcode
endpoints 0x81/0x02.

### Tier 3 — hardware timestamping

**3.1 — It already exists, it is already Linux-ABI-correct, and we only have to enable it.**

- The class advertises `GS_USB_CAN_FEATURE_HW_TIMESTAMP` when `CONFIG_USBD_GS_USB_TIMESTAMP` is set
  and the app registered a `timestamp` op (`gs_usb_features_from_ops()`, `:1423-1425`). The host
  opts in with `GS_USB_CAN_MODE_HW_TIMESTAMP`.
- A little-endian 32-bit timestamp is appended to RX frames (`:1087-1091`), **TX echo frames**
  (`:1198-1202`) and error frames (`:996-1000`), plus the `GS_USB_REQUEST_TIMESTAMP` control read.
- The kernel does the hard part: `cyclecounter`/`timecounter` over the 32-bit value with a delayed
  work item rescheduling every 1800 s to catch the ~71.6 minute wrap, surfacing real
  `skb_hwtstamps()` to SocketCAN (`SO_TIMESTAMPING`).
- The **exactly 1 MHz, exactly 32-bit** requirement that `app/src/timestamp-counter.c:34-42`
  enforces is a kernel ABI constant (`GS_USB_TIMESTAMP_TIMER_HZ`), not an arbitrary CANnectivity
  choice. Do not try to relax it.

For G474, point the `timestamp-counter` phandle at TIM2 (32-bit on G4) with the prescaler set for
1 MHz. At a 170 MHz timer clock that is a PSC value of 169 — `st,prescaler` is the raw register
value, i.e. divisor − 1. Modelled on `app/boards/candlelightfd_stm32g0b1xx.overlay`:

```dts
/ {
	cannectivity: cannectivity {
		compatible = "cannectivity";
		timestamp-counter = <&counter2>;

		channel0 {
			compatible = "cannectivity-channel";
			can-controller = <&fdcan1>;
		};
	};
};

&timers2 {
	status = "okay";
	st,prescaler = <169>;   /* 170 MHz / 170 = 1 MHz; adjust to our actual timer clock */

	counter2: counter2 {
		compatible = "st,stm32-counter";
		status = "okay";
	};
};
```

`CONFIG_CANNECTIVITY_TIMESTAMP_COUNTER` then defaults to `y` because the phandle exists
(`app/Kconfig`), which selects `CANNECTIVITY_TIMESTAMP` → `USBD_GS_USB_TIMESTAMP`.
`cannectivity_timestamp_init()` validates the frequency and width at boot and refuses to start if
the prescaler is wrong, so a mistake here fails loudly rather than silently producing garbage.

Honest cost: **+4 bytes on every host frame** — note `GS_USB_TIMESTAMP_SIZE` is compile-time
(`include/cannectivity/usb/class/gs_usb.h:474-480`), so frames grow whether or not the host asked
for timestamps — plus a `counter_get_value()` inside the CAN RX ISR.

**3.2 — Accuracy upgrade (new work, worth scoping separately).**

Today the timestamp is sampled *in software, inside the CAN RX callback*
(`gs_usb_can_rx_callback()`, `:1032-1042`), so it carries the full CAN ISR latency as jitter —
microseconds normally, considerably worse whenever the LED workqueue or the logging thread
preempts (§2.2).

Meanwhile **the M_CAN peripheral already captures a per-frame reception timestamp in hardware**,
and Zephyr already exposes it: with `CONFIG_CAN_RX_TIMESTAMP` enabled, `can_mcan.c` sets
`frame.timestamp = hdr.rxts` from the RX FIFO element header. CANnectivity ignores this field
completely.

Proposed approach: use `rxts` as a **correction** to the free-running 1 MHz counter rather than a
replacement. M_CAN's timestamp counter is 16-bit and counts CAN bit times, so it wraps in ~65 ms at
1 Mbit and cannot stand alone against a 32-bit 1 MHz ABI — but the delta between `rxts` at
reception and `rxts` at ISR entry pins down exactly how long ago the frame arrived, which is
precisely the jitter we want removed. Result: jitter-free microsecond timestamps that still satisfy
the kernel's 1 MHz/32-bit contract.

This needs a new op plumbed through the gs_usb class (the current `gs_usb_ops.timestamp` signature
has no frame context) and is genuinely new work, not a config change.

Also worth checking against RM0440: whether G4's M_CAN can source its timestamp counter externally
(`TSCC.TSS = 10`) from a TIM. If so, the correction step disappears and the hardware timestamp
becomes directly usable.

**3.3 — Leave `CONFIG_USBD_GS_USB_TIMESTAMP_SOF` off.** It latches the timestamp on every USB
Start-of-Frame, i.e. a 1 kHz counter read in USB interrupt context, and its own Kconfig help admits
*"with the cost of a higher CPU load."* 3.2 gives better accuracy for less.

### Explicitly not worth pursuing

- **Batching multiple CAN frames into one URB** — breaks the mainline Linux driver. One host frame
  per URB, both directions, no exceptions.
- **Shrinking FD host frames to the actual DLC** — the protocol fixes them at 80 bytes.
- **USB high speed** — no HS controller on G474. If we ever need to go past what FS allows, that is
  a hardware decision (an HS-capable part) or a protocol decision, not a firmware one.
- Be aware of [zephyr#93704](https://github.com/zephyrproject-rtos/zephyr/issues/93704), *"USB
  device next stack poor performance"* — the `device_next` stack measured roughly 7× slower than a
  vendor stack on an LPC55S69 at high speed. Closed as low priority.
  [zephyr#99779](https://github.com/zephyrproject-rtos/zephyr/issues/99779) (CDC ACM regression) is
  still open. After Tier 1, the Zephyr USB stack itself may well be the next wall we hit.

---

## 6. Suggested order of work

1. Rebase onto upstream `v1.4.0` or later.
2. Tier 0 (build config) and Tier 2.1/2.3 (board overlay). Measure — this is the honest "does
   CANnectivity work for us at all" baseline, and it costs almost nothing to get.
3. Tier 1.2 (freeze fix). Correctness before speed; also makes load testing survivable.
4. Tier 1.1 + 1.4 (IN pipelining and pool split). Measure again.
5. Tier 1.3 and 2.2 if the numbers still fall short.
6. Tier 3.1 (enable timestamping) at any point — it is independent. Tier 3.2 only if measured
   timestamp jitter turns out to matter for our application.

---

## 7. Comparison summary

| | CANnectivity (today) | CANnectivity (after Tier 0+1) | candleLight_fw / FD |
|---|---|---|---|
| Structure | Two threads + FIFOs on an RTOS | same | Bare-metal 7-step polling loop, nothing blocks |
| IN transfers in flight | **1** | 4–8 | Re-armed from the main loop, no thread hop |
| Frame buffers | 20, shared RX+TX+all channels | 48–64, split | 64 **per channel**, zero-copy list handoff |
| Pool exhaustion | Drop + overflow flag; OUT pipe can die permanently | Back-pressure, recoverable | Stops arming OUT → host NAK, no loss |
| TX with no ACK | `K_FOREVER`, wedges **all** channels | Bounded timeout, per-channel | Has its own variant of this bug (#58) |
| TX echo timestamp | **At real TX completion** (correct) | same | At hand-off (incorrect, #175) |
| CAN FD | Yes | Yes | Yes (G0B1 builds) |
| CPU headroom | 170 MHz M4F | 170 MHz M4F | 64 MHz M0+ (candleLightFD) |

The wire protocol ceiling is identical for all three columns.

---

## 8. How to measure

The estimates in §5 should be replaced by numbers. Baseline every run against our current
candlelight-fd build on the same hardware and the same host.

**Throughput**

- RX only: flood from a second adapter with `cangen canX -g 0 -i`, count received frames on the
  device under test over a fixed window (`candump -n`, or `timeout 30 candump canX | wc -l`).
- TX only: `cangen` on the device under test into a bus with a real ACKing peer.
- Bidirectional: both at once — this is the case that froze, and the case that matters.
- Sweep classic 1 Mbit and FD 1M/5M, DLC 0 and DLC 8/64, since per-frame overhead dominates at
  small DLC and bandwidth dominates at large.
- `canfdtest` for a correctness-under-load check alongside raw rate.

**Loss and stalls**

- `ip -s -d link show canX` for `dropped` / `overrun` counters on the host side.
- Device-side drops surface only as `GS_USB_CAN_FLAG_OVERFLOW` on a subsequent frame — count those.
  A non-zero count means the pool ran dry (§3.4).
- M_CAN RX FIFO overruns are the other loss point; §2.1 is what moves that needle.

**The freeze repro** (from upstream #217, worth running before *and* after 1.2):

1. Two channels up. Channel 0 on a live bus with a peer, channel 1 physically disconnected.
2. Blast TX at channel 1. Expect `write: No buffer space available` on the host.
3. Confirm TX on **both** channels is then dead and stays dead until power cycle.
4. After the fix: channel 1 TX should fail with a bounded error and channel 0 should be unaffected.

**Timestamp quality** (for §3.2)

- With `SO_TIMESTAMPING` enabled, send a known-period frame from a peer and look at the
  distribution of inter-arrival timestamps. The spread is the ISR jitter that 3.2 removes.
- Run it again with the LED FSM and logging enabled to see how much §2.2 contributes.
