param([int]$Workers = 3)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$python = Join-Path $repo '.venv\Scripts\python.exe'
$collection = Join-Path $repo 'results\offroad\mobility_collection'
New-Item -ItemType Directory -Force -Path $collection | Out-Null
$env:OPENBLAS_NUM_THREADS = '1'
$env:OMP_NUM_THREADS = '1'
$env:PYTHONUNBUFFERED = '1'
$stages = @(
    @{name='showcase'; args=@('--suite','showcase','--output','results/offroad/mobility_showcase')},
    @{name='robustness'; args=@('--suite','robustness','--output','results/offroad/mobility_robustness')},
    @{name='speed'; args=@('--suite','speed','--output','results/offroad/mobility_speed')},
    @{name='half_step'; args=@('--suite','showcase','--cases','hills_circle_ccw','rough_side_to_side','rough_figure8','--timestep','0.0005','--output','results/offroad/mobility_half_step')}
)
$results = @()
foreach ($stage in $stages) {
    $name = $stage.name
    @{status='running'; stage=$name; launcher_pid=$PID; completed_stages=$results} |
        ConvertTo-Json -Depth 8 | Set-Content -Encoding UTF8 (Join-Path $collection 'collection.json')
    $arguments = @('-m','spherical_wheel.mobility','--workers',"$Workers") + $stage.args
    $process = Start-Process -FilePath $python -ArgumentList $arguments -WorkingDirectory $repo `
        -WindowStyle Hidden -Wait -PassThru `
        -RedirectStandardOutput (Join-Path $collection "$name.log") `
        -RedirectStandardError (Join-Path $collection "$name.stderr.log")
    $results += @{stage=$name; exit_code=$process.ExitCode}
    if ($process.ExitCode -ne 0) {
        @{status='failed'; stage=$name; launcher_pid=$PID; completed_stages=$results} |
            ConvertTo-Json -Depth 8 | Set-Content -Encoding UTF8 (Join-Path $collection 'collection.json')
        exit $process.ExitCode
    }
}
@{status='complete'; launcher_pid=$PID; completed_stages=$results} |
    ConvertTo-Json -Depth 8 | Set-Content -Encoding UTF8 (Join-Path $collection 'collection.json')
