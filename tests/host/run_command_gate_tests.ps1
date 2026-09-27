$ErrorActionPreference = "Stop"

$Gcc = "C:\msys64\ucrt64\bin\gcc.exe"
$TestRoot = $PSScriptRoot
$RepositoryRoot = Split-Path -Parent (Split-Path -Parent $TestRoot)
$BuildRoot = Join-Path $TestRoot ".build"
$Executable = Join-Path $BuildRoot "command_gate_tests.exe"

if (-not (Test-Path -LiteralPath $Gcc -PathType Leaf)) {
    throw "Host compiler not found: $Gcc"
}

New-Item -ItemType Directory -Path $BuildRoot -Force | Out-Null
try {
    $Combinations = @(
        @(),
        @("-DEXTERNAL_COMMANDS=1", "-DHIL_MODE=0"),
        @("-DEXTERNAL_COMMANDS=1", "-DHIL_MODE=1"),
        @("-DEXTERNAL_COMMANDS=0", "-DHIL_MODE=0")
    )
    foreach ($Combination in $Combinations) {
        & $Gcc -std=c11 -Wall -Wextra -Werror @Combination `
            "-I$TestRoot\stubs" "-I$(Join-Path $RepositoryRoot 'Core/Inc')" `
            (Join-Path $TestRoot "test_command_gate.c") -o $Executable
        if ($LASTEXITCODE -ne 0) {
            throw "Command gate test build failed"
        }
        & $Executable
        if ($LASTEXITCODE -ne 0) {
            throw "Command gate tests failed"
        }
    }
}
finally {
    if (Test-Path -LiteralPath $BuildRoot) {
        Remove-Item -LiteralPath $BuildRoot -Recurse -Force
    }
}
