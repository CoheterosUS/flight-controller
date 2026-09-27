$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$Gcc = "C:\msys64\ucrt64\bin\gcc.exe"
$Build = Join-Path $PSScriptRoot "apogee_host.exe"

if (-not (Test-Path -LiteralPath $Gcc -PathType Leaf)) {
    throw "Host compiler not found: $Gcc"
}

$Sources = @(
    (Join-Path $Root "Core\Src\Utils\ApogeeDetector.c"),
    (Join-Path $PSScriptRoot "test_apogee.c")
)

Push-Location $Root
try {
    & $Gcc -std=c11 -Wall -Wextra -Werror -DAPOGEE_DETECTOR_HOST_TEST `
        "-ICore/Inc" @Sources -lm -o $Build
    if ($LASTEXITCODE -ne 0) {
        throw "Host test build failed"
    }
    & $Build
    if ($LASTEXITCODE -ne 0) {
        throw "Host tests failed"
    }
}
finally {
    Pop-Location
    if (Test-Path -LiteralPath $Build) {
        Remove-Item -LiteralPath $Build -Force
    }
}
