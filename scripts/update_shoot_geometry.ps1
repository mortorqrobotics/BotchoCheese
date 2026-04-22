param(
    [double]$HubX = 4.633,
    [double]$HubY = 4.019,
    [double]$RadiusMeters = 2.5,
    [double]$HeadingOffsetDeg = 180.0,
    [string]$PathsDir = "src/main/deploy/pathplanner/paths"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Normalize-Degrees {
    param([double]$Degrees)
    $result = $Degrees
    while ($result -gt 180.0) { $result -= 360.0 }
    while ($result -le -180.0) { $result += 360.0 }
    return $result
}

function Get-AngleToHubDeg {
    param([double]$X, [double]$Y, [double]$CenterX, [double]$CenterY)
    return [Math]::Atan2($CenterY - $Y, $CenterX - $X) * 180.0 / [Math]::PI
}

function Set-CollinearControls {
    param($WaypointA, $WaypointB)
    $x0 = [double]$WaypointA.anchor.x
    $y0 = [double]$WaypointA.anchor.y
    $x1 = [double]$WaypointB.anchor.x
    $y1 = [double]$WaypointB.anchor.y

    $WaypointA.nextControl = @{
        x = $x0 + (($x1 - $x0) / 3.0)
        y = $y0 + (($y1 - $y0) / 3.0)
    }
    $WaypointB.prevControl = @{
        x = $x0 + (2.0 * (($x1 - $x0) / 3.0))
        y = $y0 + (2.0 * (($y1 - $y0) / 3.0))
    }
}

function Update-PathFile {
    param(
        [string]$FilePath,
        [int]$WaypointIndex,
        [hashtable]$AnchorByKey,
        [hashtable]$RotationByKey,
        [string]$ShootKey,
        [bool]$IsIncoming
    )

    $json = Get-Content $FilePath -Raw | ConvertFrom-Json
    $json.waypoints[$WaypointIndex].anchor.x = $AnchorByKey[$ShootKey].x
    $json.waypoints[$WaypointIndex].anchor.y = $AnchorByKey[$ShootKey].y

    if ($IsIncoming) {
        if ($WaypointIndex -ne 1) {
            throw "Incoming path '$FilePath' expected shoot waypoint index 1."
        }
        Set-CollinearControls -WaypointA $json.waypoints[0] -WaypointB $json.waypoints[1]
        $json.goalEndState.rotation = $RotationByKey[$ShootKey]
    } else {
        if ($WaypointIndex -ne 0) {
            throw "Outgoing path '$FilePath' expected shoot waypoint index 0."
        }
        Set-CollinearControls -WaypointA $json.waypoints[0] -WaypointB $json.waypoints[1]
        $json.idealStartingState.rotation = $RotationByKey[$ShootKey]
    }

    $updated = $json | ConvertTo-Json -Depth 100
    Set-Content -Path $FilePath -Value $updated -NoNewline
}

$inboundConfigs = @(
    @{ file = "Left side.path"; shootKey = "left"; waypointIndex = 1; incoming = $true },
    @{ file = "Middle.path"; shootKey = "middle"; waypointIndex = 1; incoming = $true },
    @{ file = "Right side.path"; shootKey = "right"; waypointIndex = 1; incoming = $true }
)

$outboundConfigs = @(
    @{ file = "Left side alliance balls.path"; shootKey = "left"; waypointIndex = 0; incoming = $false },
    @{ file = "Middle alliance balls.path"; shootKey = "middle"; waypointIndex = 0; incoming = $false },
    @{ file = "Right side human player.path"; shootKey = "right"; waypointIndex = 0; incoming = $false }
)

# Use inbound path shoot anchors as canonical angle sources.
$canonicalAnglesDeg = @{}
foreach ($cfg in $inboundConfigs) {
    $fullPath = Join-Path $PathsDir $cfg.file
    $json = Get-Content $fullPath -Raw | ConvertFrom-Json
    $shootWp = $json.waypoints[$cfg.waypointIndex]
    $anchorX = [double]$shootWp.anchor.x
    $anchorY = [double]$shootWp.anchor.y
    $theta = [Math]::Atan2($anchorY - $HubY, $anchorX - $HubX)
    $canonicalAnglesDeg[$cfg.shootKey] = $theta * 180.0 / [Math]::PI
}

$anchors = @{}
$rotations = @{}

foreach ($shootKey in @("left", "middle", "right")) {
    $thetaDeg = $canonicalAnglesDeg[$shootKey]
    $thetaRad = $thetaDeg * [Math]::PI / 180.0
    $x = $HubX + ($RadiusMeters * [Math]::Cos($thetaRad))
    $y = $HubY + ($RadiusMeters * [Math]::Sin($thetaRad))

    $anchors[$shootKey] = @{ x = $x; y = $y }

    $toHubDeg = Get-AngleToHubDeg -X $x -Y $y -CenterX $HubX -CenterY $HubY
    $rotations[$shootKey] = Normalize-Degrees ($toHubDeg + $HeadingOffsetDeg)
}

foreach ($cfg in ($inboundConfigs + $outboundConfigs)) {
    $fullPath = Join-Path $PathsDir $cfg.file
    Update-PathFile `
        -FilePath $fullPath `
        -WaypointIndex $cfg.waypointIndex `
        -AnchorByKey $anchors `
        -RotationByKey $rotations `
        -ShootKey $cfg.shootKey `
        -IsIncoming $cfg.incoming
}

Write-Output ("Updated shoot geometry: center=({0}, {1}), radius={2}m" -f $HubX, $HubY, $RadiusMeters)
Write-Output ("left   anchor=({0:N6}, {1:N6}) heading={2:N6}" -f $anchors.left.x, $anchors.left.y, $rotations.left)
Write-Output ("middle anchor=({0:N6}, {1:N6}) heading={2:N6}" -f $anchors.middle.x, $anchors.middle.y, $rotations.middle)
Write-Output ("right  anchor=({0:N6}, {1:N6}) heading={2:N6}" -f $anchors.right.x, $anchors.right.y, $rotations.right)
