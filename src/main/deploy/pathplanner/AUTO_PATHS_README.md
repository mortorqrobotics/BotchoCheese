# Auto Path Quick Reference

This document explains how autonomous paths are structured and how to keep shoot geometry consistent.

## Hub Geometry (Blue)

- Hub center: `(4.633, 4.019)` meters
- Master shoot radius (robot center to hub center): `2.5 m`
- Tuning target for future events: change this one value first, then update all shoot anchors.

Shoot anchors are kept on this radius for consistency:
- `Left shoot`: `(3.18347455124566, 6.05587897858796)`
- `Middle shoot`: `(2.133, 4.019)`
- `Right shoot`: `(3.18347455124566, 1.98212102141204)`

## Shoot Heading Convention

- Middle shoot heading: `180 deg`
- Left/right shoot headings are mirrored:
  - Left: `125.437192784596 deg`
  - Right: `-125.437192784596 deg`

## Path Files

Primary "drive-to-shoot" paths:
- `paths/Left side.path`
- `paths/Middle.path`
- `paths/Right side.path`

Post-shoot follow-up paths:
- `paths/Left side alliance balls.path`
- `paths/Middle alliance balls.path`
- `paths/Right side human player.path`

Auto wrappers:
- `autos/Left side shoot.auto`
- `autos/Middle shoot.auto`
- `autos/Right side shoot.auto`

## Current Auto Flow

- Left auto: `Left side` -> `Shoot` -> `Left side alliance balls`
- Middle auto: `Middle` -> `Shoot` -> `Middle alliance balls`
- Right auto: `Right side` -> `Shoot`

## Editing Rules

1. If you move any shoot anchor, update all paths that reference that shoot location.
2. Keep left/right mirrored whenever possible.
3. Keep all shoot anchors on the same master radius unless intentionally testing a new distance.
4. If PathPlanner behaves oddly, validate that each `.path` file remains valid JSON.

## Future Distance Tuning (Meters)

When you want to test a new auto shoot distance:

1. Pick new radius `R` in meters.
2. Keep each shoot point's current angle around hub.
3. Recompute each shoot anchor from:

`x = hubX + R * cos(theta)`

`y = hubY + R * sin(theta)`

4. Update these six path files:
- `paths/Left side.path`
- `paths/Middle.path`
- `paths/Right side.path`
- `paths/Left side alliance balls.path`
- `paths/Middle alliance balls.path`
- `paths/Right side human player.path`
5. Verify headings still match your shooting convention (`middle = 180`, left/right mirrored).

Tip:
- If you prefer tuning by front-bumper-to-hub distance instead of robot-center distance, convert first:
- `R_center = R_front + centerToFrontBumperMeters`

### One-Command Updater

Use the script below to update all shoot anchors and shoot headings consistently:

`powershell -ExecutionPolicy Bypass -File scripts/update_shoot_geometry.ps1 -RadiusMeters 2.5`

Optional parameters:
- `-HubX` and `-HubY` to change hub center
- `-HeadingOffsetDeg` if your heading convention changes

Examples:
- `powershell -ExecutionPolicy Bypass -File scripts/update_shoot_geometry.ps1 -RadiusMeters 2.35`
- `powershell -ExecutionPolicy Bypass -File scripts/update_shoot_geometry.ps1 -RadiusMeters 2.6`

## One-Line Radius Check

Use this to verify a shoot point `(x, y)` is at the target radius:

`radius = sqrt((x - 4.633)^2 + (y - 4.019)^2)`

Target is `2.5`.
