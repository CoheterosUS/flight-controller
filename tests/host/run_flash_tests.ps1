$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$Compiler = "C:\msys64\ucrt64\bin\gcc.exe"
$Output = Join-Path $PSScriptRoot "flash_tests.exe"

if (-not (Test-Path -LiteralPath $Compiler -PathType Leaf)) {
    Write-Error "Compiler not found: $Compiler"
}

Push-Location $Root
try {
    & $Compiler -std=c11 -Wall -Wextra -Werror `
        -Itests/host/include `
        tests/host/flash_sim.c `
        Core/Src/Sensors/W25Q32JV/W25Q32JVCal.c `
        tests/host/test_cal.c `
        -o $Output -lm
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $Output
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
