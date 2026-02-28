<# 
bench_mmap_pipeline.ps1

End-to-end confirmation pipeline (PowerShell):
  1) Record required .mmap/.mmtile/.map files using original on-disk loader.
  2) Convert ONLY those files into the sharded Lua-blob addon format.
  3) Compare computeDistance results between original loader and Lua loader.

Usage:
  pwsh .\bench_mmap_pipeline.ps1 -OrigDataDir "D:\data" -MapId 0 -OrderingsFile ".\orderings.lua" -AddonsOutDir ".\generated_addons" -AddonPrefix "qhstub_mmapdata"
#>

param(
  [Parameter(Mandatory=$true)][string]$OrigDataDir,
  [int]$MapId = 0,
  [string]$OrderingsFile = "orderings.lua",
  [string]$AddonsOutDir = ".\generated_addons",
  [string]$AddonPrefix = "qhstub_mmapdata",
  [string]$LuaExe = "",
  [string]$PythonExe = "python"
)

$ErrorActionPreference = "Stop"

function Invoke-Checked {
  param(
    [Parameter(Mandatory=$true)][string]$Exe,
    [Parameter(Mandatory=$true)][string[]]$Args
  )
  & $Exe @Args
  if ($LASTEXITCODE -ne 0) {
    throw "Command failed ($LASTEXITCODE): $Exe $($Args -join ' ')"
  }
}

if (-not $LuaExe -or $LuaExe -eq "") {
  $candidate = "C:\Program Files (x86)\Lua\5.1\lua.exe"
  if (Test-Path $candidate) {
    $LuaExe = $candidate
  } else {
    $LuaExe = "lua"
  }
}

Write-Host "=== Step 1: record required files ==="
Invoke-Checked $LuaExe @(".\bench_mmap_manifest.lua",$OrderingsFile,$OrigDataDir,$MapId,"--present-out","required_present.txt","--missing-out","required_missing.txt")

Write-Host ""
Write-Host "=== Step 2: convert required files to Lua blobs ==="
Invoke-Checked $PythonExe @(".\tools\convert_mmaps_to_lua.py","--input-data-dir",$OrigDataDir,"--output-addons-dir",$AddonsOutDir,"--addon-prefix",$AddonPrefix,"--manifest","required_present.txt","--lua-safe-ascii")

Write-Host ""
Write-Host "=== Step 3: byte-compare required files (disk vs lua loader) ==="
Invoke-Checked $LuaExe @(".\bench_mmap_bytes_compare.lua","required_present.txt",$OrigDataDir,$AddonsOutDir,"--addon-prefix",$AddonPrefix)

Write-Host ""
Write-Host "=== Step 4: compare original vs lua loader (computeDistance) ==="
Invoke-Checked $LuaExe @(".\bench_mmap_compare.lua",$OrderingsFile,$OrigDataDir,$MapId,$AddonsOutDir,"--addon-prefix",$AddonPrefix)

Write-Host ""
Write-Host "=== OK: loaders match ==="

