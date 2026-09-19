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
