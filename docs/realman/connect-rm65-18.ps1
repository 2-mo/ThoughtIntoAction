# Run on Windows PowerShell, not in the Linux SSH terminal.
$ErrorActionPreference = 'Stop'
$SshTarget = 'guest@100.126.219.88'
$RobotAddress = '192.168.1.18'
$LocalAddress = '127.0.0.1'

if (-not (Get-Command ssh.exe -ErrorAction SilentlyContinue)) {
    throw 'Windows OpenSSH Client (ssh.exe) is required.'
}

Write-Host "Robot: $RobotAddress"
Write-Host "Open http://$LocalAddress/#/login after SSH connects."
Write-Host 'Keep this window open. Press Ctrl+C to stop forwarding.'
Write-Host 'No route to host: check robot power, cable and Linux-to-robot connectivity.'

$SshArguments = @(
    '-N'
    '-o', 'ExitOnForwardFailure=yes'
    '-o', 'ServerAliveInterval=30'
    '-o', 'ServerAliveCountMax=3'
    '-L', "${LocalAddress}:80:${RobotAddress}:80"
    '-L', "${LocalAddress}:8090:${RobotAddress}:8090"
    '-L', "${LocalAddress}:8060:${RobotAddress}:8060"
    $SshTarget
)
& ssh.exe @SshArguments
exit $LASTEXITCODE
