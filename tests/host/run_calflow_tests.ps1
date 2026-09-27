$ErrorActionPreference = "Stop"

$Gcc = "C:\msys64\ucrt64\bin\gcc.exe"
$TestRoot = $PSScriptRoot
$RepositoryRoot = Split-Path -Parent (Split-Path -Parent $TestRoot)
$BuildRoot = Join-Path $TestRoot ".build"
$Executable = Join-Path $BuildRoot "calflow_tests.exe"

if (-not (Test-Path -LiteralPath $Gcc -PathType Leaf)) {
    throw "Host compiler not found: $Gcc"
}

New-Item -ItemType Directory -Path $BuildRoot -Force | Out-Null
$Sources = @(
    (Join-Path $RepositoryRoot "Core\Src\Utils\ImuCal.c"),
    (Join-Path $RepositoryRoot "Core\Src\Utils\ImuTumbleCal.c"),
    (Join-Path $RepositoryRoot "Core\Src\Kalman\Libs\arm_mat_qr_f32.c"),
    (Join-Path $RepositoryRoot "Core\Src\Kalman\Libs\arm_householder_f32.c"),
    (Join-Path $RepositoryRoot "Core\Src\Kalman\Libs\arm_dot_prod_f32.c"),
    (Join-Path $RepositoryRoot "Core\Src\Kalman\Libs\arm_scale_f32.c"),
    (Join-Path $TestRoot "test_calflow.c"),
    (Join-Path $TestRoot "test_tumble_derive.c")
)

Push-Location $RepositoryRoot
try {
    & $Gcc -std=c11 -Wall -Wextra -Werror -DIMU_CAL_HOST -DIMU_TUMBLE_HOST `
        "-ICore/Inc" "-ICore/Src/Kalman/Libs" @Sources -lm -o $Executable
    if ($LASTEXITCODE -ne 0) { throw "Calibration flow host test build failed" }
    & $Executable
    if ($LASTEXITCODE -ne 0) { throw "Calibration flow host tests failed" }
}
finally {
    Pop-Location
    if (Test-Path -LiteralPath $BuildRoot) {
        Remove-Item -LiteralPath $BuildRoot -Recurse -Force
    }
}
