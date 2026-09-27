$ErrorActionPreference = "Stop"

$Gcc = "C:\msys64\ucrt64\bin\gcc.exe"
$TestRoot = $PSScriptRoot
$RepositoryRoot = Split-Path -Parent (Split-Path -Parent $TestRoot)
$BuildRoot = Join-Path $TestRoot ".build"
$Executable = Join-Path $BuildRoot "telemetry_layout_tests.exe"
$Source = Join-Path $TestRoot "test_telemetry_layout.c"
$Includes = @("-I$TestRoot\stubs", "-I$(Join-Path $RepositoryRoot 'Core/Inc')")

if (-not (Test-Path -LiteralPath $Gcc -PathType Leaf)) {
    throw "Host compiler not found: $Gcc"
}

New-Item -ItemType Directory -Path $BuildRoot -Force | Out-Null
try {
    foreach ($HilMode in @("0", "1")) {
        & $Gcc -std=c11 -Wall -Wextra -Werror "-DHIL_MODE=$HilMode" @Includes $Source -o $Executable
        if ($LASTEXITCODE -ne 0) {
            throw "Telemetry layout test build failed (HIL_MODE=$HilMode)"
        }
        & $Executable
        if ($LASTEXITCODE -ne 0) {
            throw "Telemetry layout tests failed (HIL_MODE=$HilMode)"
        }
    }

    # Negative probe: in a flight build (HIL_MODE 0) the HIL packet type must not exist.
    # stderr is merged into the output, so native stderr lines must not stop the script (PowerShell 5.1).
    $ErrorActionPreference = "Continue"
    $ProbeOutput = & $Gcc -std=c11 -Wall -Wextra -fsyntax-only "-DHIL_MODE=0" -DPROBE_HIL_TYPE @Includes $Source 2>&1 |
        Out-String
    $ProbeExit = $LASTEXITCODE
    $ErrorActionPreference = "Stop"
    if ($ProbeExit -eq 0) {
        throw "TelemetryPacketHil_t exists in a HIL_MODE=0 build"
    }
    if ($ProbeOutput -notmatch "TelemetryPacketHil_t") {
        throw "HIL_MODE=0 probe failed for an unexpected reason: $ProbeOutput"
    }
    Write-Host "HIL_MODE=0: TelemetryPacketHil_t is not defined (probe build failed as expected)"
}
finally {
    if (Test-Path -LiteralPath $BuildRoot) {
        Remove-Item -LiteralPath $BuildRoot -Recurse -Force
    }
}
