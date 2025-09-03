<#
.SYNOPSIS
Builds (and optionally runs) the CH_Demo project.

.DESCRIPTION
Configures, builds, and optionally runs the CH_Demo project using CMake.

.PARAMETER BuildType
Debug or Release (default Debug)

.PARAMETER Clean
Remove build directory before configuring.

.PARAMETER Generator
CMake generator (e.g. "Ninja", "Visual Studio 17 2022"). If omitted, defaults to VS 2022.

.PARAMETER RunArgs
Arguments passed to the executable after build.

.PARAMETER Jobs
Number of parallel build jobs (default: number of logical cores).

.PARAMETER BuildOnly
Skip running the executable after building.

.PARAMETER BuildDir
Custom build directory path (default: "build" subdirectory).

.EXAMPLE
.\compile.ps1 -BuildType Release -Clean
#>

[CmdletBinding()]
param(
    [ValidateSet("Debug","Release")]
    [string]$BuildType = "Debug",
    [switch]$Clean,
    [string]$Generator,
    [string[]]$RunArgs,
    [int]$Jobs = 0,
    [switch]$BuildOnly,
    [string]$BuildDir
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Find-Generator {
    param(
        [string]$UserGen
    )
    if ($UserGen) { return $UserGen }
    
    # Check if Ninja is available
    if (Get-Command "ninja" -ErrorAction SilentlyContinue) {
        return "Ninja"
    }
    
    # Fall back to Visual Studio
    return "Visual Studio 17 2022"
}

# Resolve paths
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourceDir = $scriptDir

# Use custom build directory if specified
if (-not $BuildDir) {
    $BuildDir = Join-Path $sourceDir "build"
}
$buildDir = $BuildDir

# Determine parallel jobs
if ($Jobs -le 0) {
    $Jobs = [Environment]::ProcessorCount
}

$gen = Find-Generator -UserGen $Generator

# Show configuration summary
Write-Host "CH_Demo Build Configuration:" -ForegroundColor Cyan
Write-Host "  Source directory: $sourceDir"
Write-Host "  Build directory:  $buildDir"
Write-Host "  Build type:       $BuildType"
Write-Host "  Generator:        $gen"
Write-Host "  Parallel jobs:    $Jobs"

# Create results directories
$resultsDir = Join-Path $sourceDir "results"
if (!(Test-Path $resultsDir)) {
    Write-Host "Creating results directory..." -ForegroundColor Green
    New-Item -ItemType Directory -Path $resultsDir | Out-Null
    New-Item -ItemType Directory -Path (Join-Path $resultsDir "part_1") | Out-Null
    New-Item -ItemType Directory -Path (Join-Path $resultsDir "part_2") | Out-Null
    New-Item -ItemType Directory -Path (Join-Path $resultsDir "part_3") | Out-Null
}

if ($Clean -and (Test-Path $buildDir)) {
    Write-Host "Cleaning build directory..." -ForegroundColor Yellow
    Remove-Item $buildDir -Recurse -Force
}

if (!(Test-Path $buildDir)) {
    Write-Host "Creating build directory..." -ForegroundColor Green
    New-Item -ItemType Directory -Path $buildDir | Out-Null
}

Write-Host "Configuring project..." -ForegroundColor Green
$cmakeConfigureArgs = @("-S", $sourceDir, "-B", $buildDir, "-G", $gen)

# Multi-config generators ignore CMAKE_BUILD_TYPE; set it only for single-config generators
if ($gen -notmatch "Visual Studio") {
    $cmakeConfigureArgs += "-DCMAKE_BUILD_TYPE=$BuildType"
}

# Add verbose output if -Verbose is used
if ($PSCmdlet.MyInvocation.BoundParameters["Verbose"]) {
    $cmakeConfigureArgs += "--verbose"
}

& cmake @cmakeConfigureArgs
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake configuration failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}

Write-Host "Building project..." -ForegroundColor Green
$cmakeBuildArgs = @("--build", $buildDir, "--config", $BuildType)

# Add parallel jobs for build
$cmakeBuildArgs += "--parallel", "$Jobs"

# Add verbose flag if requested
if ($PSCmdlet.MyInvocation.BoundParameters["Verbose"]) {
    $cmakeBuildArgs += "--verbose"
}

& cmake @cmakeBuildArgs
if ($LASTEXITCODE -ne 0) {
    Write-Error "Build failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}

# Determine executable path based on generator and build type
$exe = Join-Path $buildDir "ch_demo.exe"
$exeVS = Join-Path (Join-Path $buildDir $BuildType) "ch_demo.exe"
$exeNinja = Join-Path $buildDir "ch_demo.exe"

if (Test-Path $exeVS) { 
    $exe = $exeVS 
} elseif (Test-Path $exeNinja) {
    $exe = $exeNinja
}

if (!(Test-Path $exe)) {
    Write-Error "Executable not found: $exe"
    exit 1
}

Write-Host "Build completed successfully!" -ForegroundColor Green

if (-not $BuildOnly) {
    Write-Host "Running: $exe $RunArgs" -ForegroundColor Cyan
    & $exe @RunArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Program exited with code $LASTEXITCODE"
    }
}
