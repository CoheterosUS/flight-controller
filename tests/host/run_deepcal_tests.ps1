$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$Gcc = "C:\msys64\ucrt64\bin\gcc.exe"
$BuildRoot = Join-Path $PSScriptRoot ".build"
$Build = Join-Path $BuildRoot "deepcal_tests.exe"

if (-not (Test-Path -LiteralPath $Gcc -PathType Leaf)) {
    throw "Host compiler not found: $Gcc"
}

New-Item -ItemType Directory -Path $BuildRoot -Force | Out-Null
try {
    Push-Location $Root
    try {
        & $Gcc -std=c11 -Wall -Wextra -Werror -DIMU_TUMBLE_HOST `
            "-I$PSScriptRoot\stubs" "-ICore/Inc" "-ICore/Src/Kalman/Libs" `
            Core/Src/Utils/DeepCalGesture.c `
            Core/Src/Utils/DeepCalSequencer.c `
            Core/Src/Utils/ImuTumbleCal.c `
            Core/Src/Kalman/Libs/arm_mat_qr_f32.c `
            Core/Src/Kalman/Libs/arm_householder_f32.c `
            Core/Src/Kalman/Libs/arm_dot_prod_f32.c `
            Core/Src/Kalman/Libs/arm_scale_f32.c `
            tests/host/test_deepcal.c -lm -o $Build
        if ($LASTEXITCODE -ne 0) {
            throw "Deep calibration host test build failed"
        }
        & $Build
        if ($LASTEXITCODE -ne 0) {
            throw "Deep calibration host tests failed"
        }
    }
    finally {
        Pop-Location
    }
}
finally {
    if (Test-Path -LiteralPath $BuildRoot) {
        Remove-Item -LiteralPath $BuildRoot -Recurse -Force
    }
}
