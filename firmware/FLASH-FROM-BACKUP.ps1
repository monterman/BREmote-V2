# FLASH-FROM-BACKUP.ps1 — reflash a saved BREmote binary with NO compiling.
#   .\firmware\FLASH-FROM-BACKUP.ps1 -Board RX -Port COM15
#   .\firmware\FLASH-FROM-BACKUP.ps1 -Board RX -Port COM15 -Set "SW36-2026-09-21-fm-stations-77a04c9-FLASHED"
# Omit -Set and it lists what is available, newest first, and uses the newest.
# Takes ~30 s. SPIFFS (your config + logs) is NOT touched: nothing is written above 0x210000.
param(
  [Parameter(Mandatory=$true)][ValidateSet("RX","TX")][string]$Board,
  [Parameter(Mandatory=$true)][string]$Port,
  [string]$Set
)
$ErrorActionPreference = "Stop"
$root    = Join-Path $PSScriptRoot $Board
$esptool = "C:\Users\Andress Montero\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.2.0\esptool.exe"
if (-not (Test-Path $esptool)) { $esptool = (Get-ChildItem "C:\Users\Andress Montero\AppData\Local\Arduino15\packages\esp32\tools\esptool_py" -Recurse -Filter esptool.exe | Select-Object -First 1).FullName }

$sets = Get-ChildItem $root -Directory | Sort-Object Name -Descending
if (-not $sets) { throw "No saved sets in $root" }
Write-Host "Saved $Board sets:"; $sets | ForEach-Object { Write-Host "  $($_.Name)" }
if (-not $Set) { $Set = $sets[0].Name; Write-Host "`nNo -Set given; using the newest: $Set" }
$dir = Join-Path $root $Set
if (-not (Test-Path $dir)) { throw "Set not found: $dir" }

$app  = Get-ChildItem $dir -Filter "*.ino.bin" | Select-Object -First 1
$boot = Get-ChildItem $dir -Filter "*.bootloader.bin" | Select-Object -First 1
$part = Get-ChildItem $dir -Filter "*.partitions.bin" | Select-Object -First 1
$ba   = Join-Path $dir "boot_app0.bin"
if (-not $app) { throw "No application .ino.bin in $dir" }

Write-Host "`nFlashing $Board on $Port from $Set ..."
$args = @("--chip","esp32c3","--port",$Port,"--baud","921600","write-flash","-z","--flash-mode","dio","--flash-freq","80m","--flash-size","4MB")
if ($boot) { $args += @("0x0",  $boot.FullName) }
if ($part) { $args += @("0x8000",$part.FullName) }
if (Test-Path $ba) { $args += @("0xe000", $ba) }
$args += @("0x10000", $app.FullName)
& $esptool @args
if ($LASTEXITCODE -ne 0) { Write-Host "`nFLASH FAILED (exit $LASTEXITCODE). Try a lower baud: edit 921600 -> 460800." -ForegroundColor Red }
else { Write-Host "`nDone. Power-cycle the board, then check the boot banner." -ForegroundColor Green }
