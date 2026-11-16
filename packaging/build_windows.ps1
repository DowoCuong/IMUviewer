param([switch]$Installer)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$python = Join-Path $projectRoot '.venv310\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) { throw 'Missing .venv310 Python environment.' }
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$stage = Join-Path $projectRoot "build\portable-$stamp"
$destination = Join-Path $projectRoot 'dist'
$backup = Join-Path $projectRoot "backups\portable-$stamp"
# All directory moves stay within this project; reject redirected destinations.
if ([IO.Path]::GetFullPath($destination) -ne "$projectRoot\dist" -or
    [IO.Path]::GetFullPath((Split-Path $backup)) -ne "$projectRoot\backups") {
    throw 'Unexpected portable or backup path.'
}
if ((Test-Path -LiteralPath $destination) -and
    ((Get-Item -LiteralPath $destination).Attributes -band [IO.FileAttributes]::ReparsePoint)) {
    throw 'Portable destination must not be a junction or symlink.'
}
Push-Location $projectRoot
try {
    # PyInstaller writes normal progress to stderr; Windows PowerShell 5.1
    # must use the process exit code rather than treating that stream as fatal.
    $ErrorActionPreference = 'Continue'
    & $python -m PyInstaller --noconfirm --clean --workpath "$stage\work" --distpath "$stage\dist" 'packaging\imu_joint_viewer.spec'
    $buildExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($buildExit -ne 0) { throw "PyInstaller failed: $buildExit" }
    & $python 'packaging\release_manifest.py' "$stage\dist\IMUJointViewer"
    if ($LASTEXITCODE -ne 0) { throw 'Portable verification/manifest failed.' }
    $exe = Join-Path $destination 'IMUJointViewer\IMUJointViewer.exe'
    if (Get-Process -Name IMUJointViewer -ErrorAction SilentlyContinue | Where-Object { $_.Path -eq $exe }) {
        throw 'Close the current IMU Viewer before publishing. The staged build is retained.'
    }
    if (Test-Path -LiteralPath $destination) { Move-Item -LiteralPath $destination -Destination $backup }
    try { Move-Item -LiteralPath "$stage\dist" -Destination $destination }
    catch {
        if ((Test-Path -LiteralPath $backup) -and -not (Test-Path -LiteralPath $destination)) {
            Move-Item -LiteralPath $backup -Destination $destination
        }
        throw
    }
    $manifest = Get-Content -LiteralPath "$destination\IMUJointViewer\release-manifest.json" -Raw | ConvertFrom-Json
    @{version=$manifest.version; portable='dist/IMUJointViewer/IMUJointViewer.exe'; launcher='RUN_IMU.cmd';
      release_manifest='dist/IMUJointViewer/release-manifest.json'; previous_portable=$backup} |
        ConvertTo-Json | Set-Content -LiteralPath "$projectRoot\CURRENT.json" -Encoding UTF8
    Write-Output "Current portable: $exe (version $($manifest.version))"
    if ($Installer) {
        $iscc = Get-Command ISCC.exe -ErrorAction SilentlyContinue
        $isccPath = if ($iscc) { $iscc.Source } else { Join-Path $env:LOCALAPPDATA 'Programs\Inno Setup 6\ISCC.exe' }
        if (-not (Test-Path -LiteralPath $isccPath)) { throw 'Inno Setup is required only for -Installer.' }
        & $isccPath "/DMyAppVersion=$($manifest.version)" 'packaging\imujoint.iss'
        if ($LASTEXITCODE -ne 0) { throw "Inno Setup failed: $LASTEXITCODE" }
    }
}
finally { Pop-Location }
