$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$Gcc = "C:\msys64\ucrt64\bin\gcc.exe"
$Build = Join-Path $PSScriptRoot "bmp581_mailbox_host.exe"

if (-not (Test-Path -LiteralPath $Gcc -PathType Leaf)) {
    throw "Host compiler not found: $Gcc"
}

Push-Location $Root
try {
    & $Gcc -std=c11 -Wall -Wextra -Werror `
        "-Itests/host/mailbox_stubs" `
        (Join-Path $PSScriptRoot "test_bmp581_mailbox.c") -o $Build
    if ($LASTEXITCODE -ne 0) {
        throw "BMP581 mailbox host test build failed"
    }
    & $Build
    if ($LASTEXITCODE -ne 0) {
        throw "BMP581 mailbox host tests failed"
    }
}
finally {
    Pop-Location
    if (Test-Path -LiteralPath $Build) {
        Remove-Item -LiteralPath $Build -Force
    }
}
