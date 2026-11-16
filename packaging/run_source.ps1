param([Parameter(ValueFromRemainingArguments=$true)][string[]]$AppArguments)
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$requirements = Join-Path $projectRoot 'requirements-desktop.txt'
$validator = Join-Path $PSScriptRoot 'check_source_runtime.py'
$runtime = Join-Path $projectRoot '.runtime'
$environment = Join-Path $runtime 'venv'
$managedPython = Join-Path $environment 'Scripts\python.exe'

function Test-SourcePython([string]$Candidate, [switch]$InterpreterOnly) {
    if (-not (Test-Path -LiteralPath $Candidate -PathType Leaf)) { return $false }
    try {
        # Native errors are a failed probe, including on Windows PowerShell 5.1.
        $ErrorActionPreference = 'Continue'
        if ($InterpreterOnly) { & $Candidate -I $validator --interpreter-only *> $null }
        else { & $Candidate -I $validator $requirements *> $null }
        return ($LASTEXITCODE -eq 0)
    }
    finally { $ErrorActionPreference = 'Stop' }
}

try {
    $python = $null
    foreach ($candidate in @((Join-Path $projectRoot '.venv310\Scripts\python.exe'),
                             (Join-Path $projectRoot '.venv\Scripts\python.exe'), $managedPython)) {
        if (Test-SourcePython $candidate) { $python = $candidate; break }
    }
    if (-not $python) {
        if ($env:PROCESSOR_ARCHITECTURE -ne 'AMD64' -and $env:PROCESSOR_ARCHITEW6432 -ne 'AMD64') {
            throw 'Ban tu dong nay can Windows 10/11 x64.'
        }
        Write-Host 'Dang chuan bi Python va thu vien. Lan dau can Internet, vui long cho...'
        if ((Test-Path -LiteralPath $runtime) -and
            ((Get-Item -LiteralPath $runtime).Attributes -band [IO.FileAttributes]::ReparsePoint)) {
            throw 'Thu muc .runtime khong duoc la junction/symlink.'
        }
        New-Item -ItemType Directory -Path $runtime -Force | Out-Null
        $uv = Join-Path $runtime 'uv\uv.exe'
        if (-not (Test-Path -LiteralPath $uv)) {
            Import-Module Microsoft.PowerShell.Archive -ErrorAction Stop
            # Pin the executable and verify the release archive before running it.
            $archive = Join-Path $runtime 'uv-0.11.3.zip'
            $url = 'https://github.com/astral-sh/uv/releases/download/0.11.3/uv-x86_64-pc-windows-msvc.zip'
            $sha256 = 'ae681c0aaec7cc96af184648cb88d73f8393ed60fa5880abdd6bdb910f9b227c'
            [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
            Write-Host 'Tai cong cu thiet lap...'
            Invoke-WebRequest -Uri $url -OutFile $archive -UseBasicParsing -TimeoutSec 180
            $hashAlgorithm = [Security.Cryptography.SHA256]::Create()
            $archiveStream = [IO.File]::OpenRead($archive)
            try { $actualHash = [BitConverter]::ToString($hashAlgorithm.ComputeHash($archiveStream)).Replace('-', '').ToLowerInvariant() }
            finally { $archiveStream.Dispose(); $hashAlgorithm.Dispose() }
            if ($actualHash -ne $sha256) {
                throw 'File tai ve khong dung SHA256. Chay lai de tai lai.'
            }
            Expand-Archive -LiteralPath $archive -DestinationPath (Join-Path $runtime 'uv') -Force
            if (-not (Test-Path -LiteralPath $uv)) { throw 'Khong tim thay uv.exe sau khi giai nen.' }
        }
        # Keep all downloaded runtime/cache files inside the ignored local folder.
        $env:UV_PYTHON_INSTALL_DIR = Join-Path $runtime 'python'
        $env:UV_CACHE_DIR = Join-Path $runtime 'cache'
        $env:UV_PYTHON_DOWNLOADS = 'automatic'
        if (-not (Test-SourcePython $managedPython -InterpreterOnly)) {
            if (Test-Path -LiteralPath $environment) {
                $oldEnvironment = Get-Item -LiteralPath $environment
                if ($oldEnvironment.FullName -ne "$runtime\venv" -or
                    ($oldEnvironment.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
                    throw 'Duong dan moi truong Python khong hop le.'
                }
                Move-Item -LiteralPath $environment -Destination (Join-Path $runtime ('venv-backup-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff')))
            }
            Write-Host 'Tai Python rieng cho app (khong thay Python cua he thong)...'
            $ErrorActionPreference = 'Continue'
            & $uv --no-config venv --managed-python --python 3.12 $environment
            $setupExit = $LASTEXITCODE
            $ErrorActionPreference = 'Stop'
            if ($setupExit -ne 0) { throw "Khong tao duoc Python: $setupExit" }
        }
        Write-Host 'Cai thu vien theo requirements-desktop.txt...'
        $ErrorActionPreference = 'Continue'
        & $uv --no-config pip install --python $managedPython --only-binary ':all:' --default-index 'https://pypi.org/simple' --requirement $requirements
        $setupExit = $LASTEXITCODE
        $ErrorActionPreference = 'Stop'
        if ($setupExit -ne 0) { throw "Chua cai xong thu vien: $setupExit" }
        if (-not (Test-SourcePython $managedPython)) { throw 'Moi truong chua dat kiem tra Python/Qt.' }
        $python = $managedPython
        Write-Host 'Thiet lap hoan tat. Cac lan sau khong can tai lai.'
    }
    Set-Location -LiteralPath $projectRoot
    $env:PYTHONNOUSERSITE = '1'
    Remove-Item Env:PYTHONHOME, Env:PYTHONPATH -ErrorAction SilentlyContinue
    $ErrorActionPreference = 'Continue'
    & $python (Join-Path $projectRoot 'run_desktop.py') @AppArguments
    exit $LASTEXITCODE
}
catch {
    Write-Host ('Khong khoi dong duoc IMU Viewer: ' + $_.Exception.Message) -ForegroundColor Red
    Write-Host 'Kiem tra Internet va quyen ghi thu muc. Bam RUN_SOURCE.cmd de thu lai.'
    exit 1
}
