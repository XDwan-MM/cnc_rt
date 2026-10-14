# M29 PLC diagnostics (2026-10-14)

## Fixture and scope

Firmware: `cnc_hmi_1.27.0.2/plc/LadData20260915.json`.
SHA256: `2322499419de64db07a45af6266b54082ce6999302b6151651e02a57afad4da7`.
The replay refuses a different fixture because its instruction ranges are fixed.

Run from the repository root:

```sh
bash test/build_m29_plc_replay.sh /absolute/path/to/LadData20260915.json
bash test/build_spindle_m29_check.sh
```

The replay uses production `PlcTask` instruction handlers with private, in-memory
registers. It executes instructions 51-82 (rigid tapping latch, completion timer,
termination) and 2050-2071 (CW command). External NC inputs and upstream interlocks
are injected. One replay scan represents 1 ms; timers 64/65 retain the fixture's
500 ms presets. No shared memory, hardware, or PLC outputs are accessed.
This is a selected-rung test, not a complete original/ported NC or machine replay.

## Results

- Cold M29, M3 then M29, and M3/M5 then M29 all reach R1.0 with healthy interlocks,
  for both constant-low and asserted F76.3 input patterns.
- Dropping F76.3 after it has latched R560.0 sets R560.1 and R500.3, dropping G61.0.
  A subsequent M29 recovers after termination clears when interlocks are healthy.
  This does NOT reproduce the persistent 30-second machine timeout by itself.
- Holding R284.0 (panel spindle stop) blocks G70.5 and R1.0; releasing it recovers.
- Holding F0.6 low reproduces the logs 10.14-7/8/9 termination pattern: G61.0
  pulses for one scan, R500.3 aborts it, and the cycle repeats after timer 65.
  R1.0 never rises although F76.3 stays high and R560.1 stays low. Restoring
  F0.6 with otherwise healthy interlocks allows completion in this replay.
- K1.3 (reverse tapping selection) blocks this fixture's CW-only completion rung.
  This is a conditional fixture behavior, not evidence that K1.3 was set on-machine.
- R1.0 feeds the M29 branch of the FIN aggregate. The replay does not execute the
  complete FIN aggregate, MF timing, or the NC's FIN latch.

## Machine capture

Use the rebuilt `bin/cnc_rt`. No environment switch is required. The diagnostic
samples after F publication, once per NC cycle; G/R values are the latest PLC
results. It prints `M29 PLC变化` on entry, phase/active changes, register changes,
or read-error changes, and continues for 2000 calls after M29 becomes inactive.
`sample` is a monotonically increasing sampling sequence, not wall-clock time.
`-1` means a register read failed, not a low input. Unchanged states are not logged.
Intra-scan intermediate states cannot be observed by this diagnostic.

The same window also prints per-axis fault-source snapshots on entry and changes:
slot 0/1/2/3 = X/Y/Z/SP, RT is_error and disabled flags, HAL read return code,
raw status word, error-code validity and error code (decimal). Failed HAL reads
print -1 for unavailable fields. This captures persistent faults whose initial
log may precede program submission; it does not change fault handling.

Compare separate runs of the already-used machine programs:

```text
M29;
M05;
```

```text
M03 S1000;
M29;
M05;
```

```text
M03 S1000;
M05;
M29;
M05;
```

Keep the log from BEFORE program submission through completion/reset and the
following 2000 cycles. Record the loaded ladder's identity and whether a reset or
prior program preceded each run. Do not force G70.5 or FIN for this test.

## Reading the trace

- R643.5, F7.0: decoded M29 and MF.
- G61.0, F76.3, R560.0/1/2, R500.3: rigid request and termination lifecycle.
- G70.5, G70.4, K1.3: CW/CCW command and tapping direction selection.
- R284.0/1: panel/M-code spindle stop.
- R1000.3, R108.4, A0.5, R212.4: clamping, estop release, cooling alarm, gear lock.
- X0.3: clamp input feeding R1000.3 in this fixture.
- F1.1, F45.0, F0.6, R500.5/7: reset, spindle fault, drive health, return conditions.
- R999.5, R282.0/1, R640.4: intermediate CW request, start pulses, M4 interlock.
- R1.0, G4.3: rigid-start completion and PLC FIN.

Trace the earliest failed condition. If G61.0 drops, inspect termination inputs.
If G61.0 stays high but G70.5 stays low, inspect CW interlocks and K1.3. If R1.0
rises but FIN does not, inspect the MF/FIN aggregate. No F76.3 control timing or
FIN acceptance rule was changed by this diagnostic patch.

## Remaining machine evidence

Logs 10.14-7/8/9 show F0.6=0, F45.0=0 and R1000.3=0. With the current NC
publisher this indicates an X/Y/Z is_error flag, but those logs do not identify
the axis or error code. Original and ported NC both derive drive health from
axis error flags; the port currently logs drive error codes without publishing
the corresponding axis alarm to HMI. Do not clear or bypass F0.6 to test this.

The loaded machine ladder still needs identity verification: log 10.14-8 includes
G70.5=1 and R999.5=1 while R1000.3=0, which does not match this fixture's CW
clamp gate with normal contacts at a settled scan. A later offline test reproduced
this combination in the SAME ladder using a forced clamp contact: instruction
state overrides the evaluated value without changing the register. Therefore the
combination alone does not prove a ladder-version mismatch. Capture live force
states as well as the loaded file; see M29_OFFLINE_AUDIT_20261014.md.
