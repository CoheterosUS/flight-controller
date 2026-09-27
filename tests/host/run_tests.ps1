$ErrorActionPreference = "Stop"

$Gcc = "C:\msys64\ucrt64\bin\gcc.exe"
$TestRoot = $PSScriptRoot
$RepositoryRoot = Split-Path -Parent (Split-Path -Parent $TestRoot)
$BuildRoot = Join-Path $TestRoot ".build"
$Executable = Join-Path $BuildRoot "buzzer_pattern_tests.exe"

if (-not (Test-Path -LiteralPath $Gcc -PathType Leaf)) {
    Write-Error "Compiler not found: $Gcc"
}

New-Item -ItemType Directory -Path $BuildRoot -Force | Out-Null
try {
    & $Gcc `
        -std=c11 `
        -Wall `
        -Wextra `
        -Werror `
        "-I$TestRoot\stubs" `
        "-I$(Join-Path $RepositoryRoot 'Core/Inc')" `
        "-o" $Executable `
        (Join-Path $TestRoot "buzzer_pattern_tests.c")

    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    & $Executable
    exit $LASTEXITCODE
}
finally {
    if (Test-Path -LiteralPath $BuildRoot) {
        Remove-Item -LiteralPath $BuildRoot -Recurse -Force
    }
}
