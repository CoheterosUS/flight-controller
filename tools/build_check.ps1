param(
    [int]$BaselineErrors = -1,
    [int]$BaselineWarnings = -1
)

$ErrorActionPreference = "Stop"
$RepositoryRoot = Split-Path -Parent $PSScriptRoot
$Compiler = $env:ARM_GCC
if ([string]::IsNullOrWhiteSpace($Compiler)) {
    $Compiler = "C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-gcc.exe"
}

if (-not (Test-Path -LiteralPath $Compiler -PathType Leaf)) {
    Write-Error "Compiler not found: $Compiler"
}

$Flags = @(
    "-mcpu=cortex-m7",
    "-mfpu=fpv5-d16",
    "-mfloat-abi=hard",
    "-mthumb",
    "-std=gnu11",
    "-Wall",
    "-DUSE_PWR_LDO_SUPPLY",
    "-DUSE_HAL_DRIVER",
    "-DSTM32H723xx",
    "-ICore/Inc",
    "-ICore/Src/Kalman",
    "-IFATFS/Target",
    "-IFATFS/App",
    "-IDrivers/STM32H7xx_HAL_Driver/Inc",
    "-IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy",
    "-IMiddlewares/Third_Party/FreeRTOS/Source/include",
    "-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
    "-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
    "-IDrivers/CMSIS/RTOS2/Include",
    "-IMiddlewares/Third_Party/FatFs/src",
    "-IDrivers/CMSIS/Device/ST/STM32H7xx/Include",
    "-IDrivers/CMSIS/Include"
)

$SourceDirectories = @(
    "Core/Src",
    "FATFS/App",
    "FATFS/Target",
    "Middlewares/Third_Party/FreeRTOS/Source"
)

$Sources = @(
    foreach ($Directory in $SourceDirectories) {
        Get-ChildItem -LiteralPath (Join-Path $RepositoryRoot $Directory) -Filter "*.c" -File -Recurse |
            Where-Object { $_.FullName -notmatch "[\\/]Core[\\/]Src[\\/]Kalman_C[\\/]" }
    }
)

$ErrorFileCount = 0
$WarningCount = 0
$FirstErrors = New-Object System.Collections.Generic.List[string]

Push-Location $RepositoryRoot
try {
    foreach ($Source in $Sources) {
        $Output = @(& $Compiler @Flags "-fsyntax-only" $Source.FullName 2>&1)
        $ExitCode = $LASTEXITCODE
        $WarningCount += @($Output | Where-Object { "$_" -match "\bwarning:" }).Count

        if ($ExitCode -ne 0) {
            $ErrorFileCount++
            if ($FirstErrors.Count -lt 10) {
                foreach ($Line in ($Output | Where-Object { "$_" -match "\berror:" })) {
                    if ($FirstErrors.Count -ge 10) {
                        break
                    }
                    $FirstErrors.Add("$($Source.FullName): $Line")
                }
            }
        }
    }
}
finally {
    Pop-Location
}

Write-Output "Files: $($Sources.Count)"
Write-Output "Errors: $ErrorFileCount"
Write-Output "Warnings: $WarningCount"
if ($FirstErrors.Count -gt 0) {
    Write-Output "First errors:"
    $FirstErrors | ForEach-Object { Write-Output "  $_" }
}

$ComparisonFailed = $false
if ($BaselineErrors -ge 0 -or $BaselineWarnings -ge 0) {
    Write-Output "Baseline: errors=$BaselineErrors warnings=$BaselineWarnings"
    if ($BaselineErrors -ge 0 -and $ErrorFileCount -ne $BaselineErrors) {
        $ComparisonFailed = $true
    }
    if ($BaselineWarnings -ge 0 -and $WarningCount -ne $BaselineWarnings) {
        $ComparisonFailed = $true
    }
}

if ($ErrorFileCount -gt 0 -or $ComparisonFailed) {
    exit 1
}

exit 0
