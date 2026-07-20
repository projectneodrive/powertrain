<#
.SYNOPSIS
    Build (and optionally run) the Qt serial plotter for WebAssembly or desktop.

.DESCRIPTION
    Wraps the environment setup that qt-cmake needs on Windows: emsdk activation,
    Qt's bundled Ninja on PATH, and the right qt-cmake wrapper per target.

.EXAMPLE
    .\build.ps1                             # build WASM
    .\build.ps1 -Run                        # build WASM, serve it, open the browser
    .\build.ps1 -Target desktop -Run        # build and launch the native app
    .\build.ps1 -Target desktop -Run -Demo  # ...with synthetic telemetry (no hardware)
    .\build.ps1 -Clean                      # wipe the build dir first
#>
[CmdletBinding()]
param(
    [ValidateSet('wasm', 'desktop')]
    [string]$Target = 'wasm',

    [switch]$Run,
    [switch]$Clean,
    [switch]$Demo,

    # Copy the WASM output into <repo>/docs for GitHub Pages.
    [switch]$Deploy,
    [string]$DeployDir,

    [string]$QtRoot = 'C:\Qt',
    [string]$QtVersion = '6.8.3',
    [string]$EmsdkRoot = 'C:\emsdk',
    [int]$Port = 8000
)

# Deliberately NOT 'Stop': cmake and qt-cmake write warnings to stderr (e.g. the
# harmless Qt6Protobuf ones), and under 'Stop' PowerShell turns any native-command
# stderr line into a terminating NativeCommandError -- failing the build over a
# warning. Success is judged by $LASTEXITCODE below instead.
$ErrorActionPreference = 'Continue'
Set-Location $PSScriptRoot

$qtBase = Join-Path $QtRoot $QtVersion
if (-not (Test-Path $qtBase)) {
    throw "Qt $QtVersion not found at $qtBase. Pass -QtRoot/-QtVersion to override."
}

# Qt ships Ninja but does not put it on PATH.
$ninjaDir = Join-Path $QtRoot 'Tools\Ninja'
if (Test-Path $ninjaDir) { $env:PATH = "$ninjaDir;$env:PATH" }

if ($Target -eq 'wasm') {
    $kit = Join-Path $qtBase 'wasm_singlethread'
    $buildDir = 'build'

    # emsdk_env.bat would set vars in a child cmd that exits; the .ps1 is the
    # one that actually affects this session.
    $emEnv = Join-Path $EmsdkRoot 'emsdk_env.ps1'
    if (-not (Test-Path $emEnv)) {
        throw "emsdk not found at $EmsdkRoot. Install it (see README) or pass -EmsdkRoot."
    }
    $env:EMSDK_QUIET = '1'
    . $emEnv
} else {
    $kit = Join-Path $qtBase 'mingw_64'
    $buildDir = 'build-desktop'

    # MinGW toolchain for the native build.
    $mingw = Get-ChildItem -Path (Join-Path $QtRoot 'Tools') -Filter 'mingw*' -Directory |
             Sort-Object Name -Descending | Select-Object -First 1
    if ($mingw) { $env:PATH = "$($mingw.FullName)\bin;$env:PATH" }
}

$qtCMake = Join-Path $kit 'bin\qt-cmake.bat'
if (-not (Test-Path $qtCMake)) {
    throw "qt-cmake not found at $qtCMake. Is the '$Target' kit installed?"
}

if ($Clean -and (Test-Path $buildDir)) {
    Write-Host "Removing $buildDir..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force $buildDir
}

# Configure only when needed; a stale cache pins the old generator.
if (-not (Test-Path (Join-Path $buildDir 'CMakeCache.txt'))) {
    Write-Host "Configuring ($Target)..." -ForegroundColor Cyan
    & $qtCMake -S . -B $buildDir -G Ninja
    if ($LASTEXITCODE -ne 0) { throw "Configure failed." }
}

Write-Host "Building ($Target)..." -ForegroundColor Cyan
$sw = [Diagnostics.Stopwatch]::StartNew()
cmake --build $buildDir
if ($LASTEXITCODE -ne 0) { throw "Build failed." }
$sw.Stop()
Write-Host ("Build OK in {0:N1}s" -f $sw.Elapsed.TotalSeconds) -ForegroundColor Green

if ($Deploy) {
    if ($Target -ne 'wasm') {
        throw "-Deploy only applies to the wasm target."
    }
    if (-not $DeployDir) {
        # docs/ already holds the project's markdown documentation, so the app
        # goes in its own subfolder rather than scattering 20+ MB of build
        # output over it (and a .nojekyll at the docs root would stop GitHub
        # rendering those .md files).
        $DeployDir = Join-Path (Resolve-Path (Join-Path $PSScriptRoot '..\..')) 'docs\plotter'
    }
    New-Item -ItemType Directory -Force -Path $DeployDir | Out-Null

    # Only the runtime files -- not CMake's build internals.
    $payload = @(
        'index.html', '.nojekyll', 'qtloader.js', 'qtlogo.svg',
        'serial_plotter_wasm.js', 'serial_plotter_wasm.wasm'
    )
    foreach ($f in $payload) {
        $src = Join-Path $buildDir $f
        if (-not (Test-Path $src)) { throw "Missing build artifact: $src" }
        Copy-Item $src -Destination $DeployDir -Force
    }
    $mb = [math]::Round(((Get-ChildItem $DeployDir -File | Measure-Object Length -Sum).Sum / 1MB), 1)
    Write-Host "Deployed to $DeployDir ($mb MB)" -ForegroundColor Green
    Write-Host "Commit it, then set GitHub Pages to deploy from that branch's /docs folder." -ForegroundColor Yellow
}

if (-not $Run) { return }

if ($Target -eq 'desktop') {
    # The exe needs Qt's DLLs at runtime; without this it exits instantly and
    # silently (Windows shows no missing-DLL dialog for a console-less launch).
    $env:PATH = "$kit\bin;$env:PATH"
    Write-Host "Launching native app$(if ($Demo) { ' (demo mode)' })..." -ForegroundColor Cyan
    $appArgs = @()
    if ($Demo) { $appArgs += '--demo' }
    & (Join-Path $buildDir 'serial_plotter_wasm.exe') @appArgs
} else {
    # Web Serial requires a secure context: localhost counts, file:// does not.
    # serve.py disables caching so a rebuilt .wasm is actually picked up.
    $url = "http://localhost:$Port/serial_plotter_wasm.html"
    Write-Host "Serving at $url (Ctrl+C to stop)" -ForegroundColor Cyan
    Start-Process $url
    python serve.py $Port
}
