# Runs every host test suite (tests/host/run_*_tests.ps1). Exits non-zero if any suite fails.
$ErrorActionPreference = "Stop"
$Failed = @()
foreach ($Suite in (Get-ChildItem -Path $PSScriptRoot -Filter "run_*_tests.ps1" | Sort-Object Name)) {
    Write-Host "== $($Suite.Name)"
    & powershell -NoProfile -ExecutionPolicy Bypass -File $Suite.FullName
    if ($LASTEXITCODE -ne 0) {
        $Failed += $Suite.Name
    }
}
if ($Failed.Count -gt 0) {
    Write-Host "FAILED: $($Failed -join ', ')"
    exit 1
}
Write-Host "All host test suites passed"
