# Lessons Learned

## 1. Bench versus implementation

Treat bench and installed hardware as two distinct configurations and document every change between them.

The original relay board likely would have worked correctly if the installed 5 V supply had remained above its minimum input voltage during engine crank. The bench setup masked the weakness because USB power was supporting the Nano. When moving from bench to machine, explicitly record changes in power source, grounding, wiring length, connectors, filtering, loads, and startup/crank conditions.

**Rule:** before redesigning a circuit that worked on the bench, first identify what changed between the bench and the installed environment.

## 2. Prototype toward the end-state

As familiarity with the boards and interfaces improves, build prototypes closer to the expected final architecture instead of repeatedly rebuilding broad experimental versions.

Experimentation is still useful when uncertainty is real, but time is limited. Prefer targeted experiments that answer a specific unknown while preserving the likely final pinout, power architecture, connectors, and physical packaging.

**Rule:** use experiments to retire uncertainty, not as a default construction method.

## 3. Verify connections before implementation

Before soldering or installing, compare every connection against either the current schematic/pin map or a deliberate second-person review.

This includes:
- GPIO numbers and physical header locations
- TX versus RX direction
- logic-level compatibility
- supply voltage
- grounds
- CANH/CANL
- SPI CS/MOSI/SCK/MISO
- connector orientation

The SD-card GPIO issue demonstrated the value of treating the pin map as a controlled engineering artifact rather than relying on memory.

**Rule:** 100% pin-by-pin preflight before permanent implementation.

## 4. Git is the way

Firmware, pin maps, architecture notes, test changes, and known-good states belong in Git.

Git provides:
- a known-good rollback point
- an exact record of what changed
- a shared source of truth between bench and field work
- easier recovery after experimental changes
- less dependence on memory or local copies

**Rule:** make Git the system of record for firmware and technical documentation, and pull before building/flashing when remote changes have been made.

## 5. Design power for the worst transient, not the nominal load

The important specification on the original 5 V converter was not its 5 A output rating; it was the roughly 8 V minimum input requirement. Crank voltage repeatedly fell into the 7.6-8.3 V range, exactly where the converter could drop out.

That created symptoms that looked like relay, firmware, CAN, or state-machine failures even though the underlying problem was power integrity.

**Rule:** qualify every supply against the worst source-voltage sag, surge, and startup transient it will actually see in service.

## 6. USB can hide a power problem

The laptop and later a USB battery bank silently supported the Nano's 5 V rail. That made the system behave correctly while connected to diagnostic equipment and fail when supposedly running standalone.

Opening or closing a serial monitor can also reset a microcontroller through DTR/RTS, creating another hidden difference between "being tested" and "being unattended."

**Rule:** include a true headless, no-USB power-cycle test before declaring embedded hardware field-ready.

## 7. Measure at the point of failure

Static voltage readings were not enough to explain the original controller problem. Logging the battery rail during crank exposed the converter-input violation immediately.

Likewise, the SD problem became tractable once power, regulator output, continuity, cards, modules, connectors, and finally GPIO assignments were tested individually.

**Rule:** when a problem is event-driven, instrument the event itself rather than relying on idle measurements.

## 8. Change one common factor at a time

Two SD cards, two HW-125 modules, new connectors, verified 5.1 V input, verified 3.3 V regulator output, and lower SPI speed all failed identically. Replacing more of the same hardware was no longer informative. Remapping SPI to GPIO4-7 changed a remaining common factor and immediately solved the problem.

**Rule:** after two equivalent replacements fail the same way, stop swapping parts and identify the common path shared by every failure.

## 9. Do not over-trust a nominally valid pin assignment

GPIO10-13 were logically reasonable and worked in an earlier configuration, yet the installed build became reliable only after moving SD SPI to GPIO4-7.

A pin can be "valid" in a datasheet sense and still be a poor choice on a specific carrier, boot configuration, peripheral mix, or physical implementation.

**Rule:** freeze only pin assignments that have been demonstrated on the exact board, firmware, and installed hardware configuration.

## 10. Separate safety authority from telemetry

The Nano remains the deterministic compressor controller. The ESP32-S3 observes, logs, publishes, and later uploads data, but it is not required for the compressor to start, run, unload, stop, or protect itself.

The one-way Nano TX -> ESP RX connection physically reinforces that boundary.

**Rule:** keep safety-critical control local and deterministic; let networking, logging, dashboards, and analytics fail without affecting the machine.

## 11. Let the OEM controller do the job it already knows how to do

The engine controller already owns automatic crank/retry behavior. Trying to infer success too early caused a failed catch/die attempt to look like an engine-loss fault. The better design was one Start/Stop request, detect starter activity, then give the OEM controller a generous retry window and separately confirm true running RPM.

**Rule:** integrate with an existing controller at the highest useful abstraction level instead of reimplementing behavior it already handles reliably.

## 12. Distinguish attempted action from confirmed state

A starter spinning is not the same thing as an engine running. A single CAN RPM transient is not proof of sustained operation. A command to energize an output is not proof that the downstream machine reached the intended state.

The controller became much more robust after adding sustained thresholds and separate events for CRANK and START CONFIRMED.

**Rule:** model command, activity, and confirmed state as separate things.

## 13. Debounce and persistence belong in state machines

Pressure-switch flicker near cutout, one-off RPM values, and temporary RPM loss all produced misleading transitions when treated as instantaneous truth.

Short confirmation windows solved these without making the controller feel sluggish.

**Rule:** any physical or network signal capable of causing a major state transition should usually require persistence, hysteresis, or both.

## 14. A monitor input is not necessarily a control interlock

The OEM master-monitor input only reflected the physical rocker. It did not reflect the parallel relay path used by the automated controller. Treating it as a startup interlock therefore blocked valid automated operation.

**Rule:** define every input by what it physically measures, not by what its name suggests.

## 15. Test subsystems independently before claiming the whole path works

The synthetic SD stress test proved only ESP -> SPI -> SD throughput. It did not test the CAN transceiver. MQTT connectivity did not prove Nano UART reception. TWAI initialization did not prove CANH/CANL capture.

The first real run finally established each path separately.

**Rule:** state exactly what a test proves and what it does not prove.

## 16. Instrument first, optimize second

The project improved quickly once raw evidence existed: crank-voltage logs, Nano status lines, SD files, MQTT health counters, CAN frame logs, and timestamped events.

Those records turned subjective symptoms into measurable transitions and made later CAN reverse engineering possible.

**Rule:** preserve raw telemetry before trying to optimize behavior; future questions are often answerable only because the data already exists.

## 17. Local storage is authoritative; networking is opportunistic

The ESP can lose Wi-Fi, MQTT, or internet time synchronization without losing the core raw run record. Monotonic timestamps start immediately; NTP enriches later records when available.

**Rule:** when evidence matters, capture it locally first and treat wireless delivery as a convenience layer.

## 18. Diagnostic counters must not depend on the subsystem they diagnose

The first Nano UART line counter incremented only inside the SD logging function. When SD failed, MQTT misleadingly suggested that UART reception was also dead even though serial monitoring showed perfect Nano status lines.

**Rule:** health counters and observability paths must remain independent of the subsystem whose failure they are meant to expose.

## 19. Prefer controlled interventions for reverse engineering

The deliberate forced-unload interval created a known state change in an otherwise steady run. That made it possible to separate actual RPM from what appears to be a target/setpoint RPM and identify several other strongly correlated CAN fields.

**Rule:** one deliberate change at a known time is often worth more than hours of uncontrolled passive data.

## 20. Preserve a safe observation mode before considering control

The ESP CAN interface started in TWAI listen-only mode. That allowed real bus capture and protocol study without ACKing, arbitrating, or transmitting onto the engine network.

**Rule:** when reverse engineering a live control bus, build a trustworthy passive observer before even considering transmission.

## 21. Build for unattended operation, then use the laptop as a microscope

The final success criterion was not "works while PlatformIO is attached." It was a completely autonomous start/run/unload/stop cycle with no laptop or USB battery supporting either controller.

**Rule:** development tools should observe the finished machine, not be part of the conditions required for it to work.

## 22. Simple faults deserve simple checks first

Several long troubleshooting branches ultimately had simple causes: the original buck falling below its input range, the SD rail temporarily having no power because the buck had been unplugged, and the SD subsystem ultimately needing a different GPIO set.

**Rule:** before escalating to exotic explanations, re-check power present, ground present, pin assignment, connector orientation, and what changed most recently.
