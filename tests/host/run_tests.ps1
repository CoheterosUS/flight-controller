$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$Gcc = "C:\msys64\ucrt64\bin\gcc.exe"
$Build = Join-Path $PSScriptRoot "imu_tumble_host.exe"

if (-not (Test-Path -LiteralPath $Gcc -PathType Leaf)) {
    throw "Host compiler not found: $Gcc"
}

$Sources = @(
    (Join-Path $Root "Core\Src\Utils\ImuTumbleCal.c"),
    (Join-Path $Root "Core\Src\Kalman\Libs\arm_mat_qr_f32.c"),
    (Join-Path $Root "Core\Src\Kalman\Libs\arm_householder_f32.c"),
    (Join-Path $Root "Core\Src\Kalman\Libs\arm_dot_prod_f32.c"),
    (Join-Path $Root "Core\Src\Kalman\Libs\arm_scale_f32.c"),
    (Join-Path $PSScriptRoot "test_imu_tumble.c")
)

Push-Location $Root
try {
    & $Gcc -std=c11 -Wall -Wextra -Werror -DIMU_TUMBLE_HOST `
        "-ICore/Inc" "-ICore/Src/Kalman/Libs" @Sources -lm -o $Build
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
