$ErrorActionPreference = "Stop"

$projectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$buildDir = Join-Path $projectDir "Debug"
$objDir = Join-Path $buildDir "obj"
$syscfgDir = Join-Path $buildDir "syscfg"
$sdkDir = "D:\ti\ccs2051\mspm0_sdk_2_10_00_04"
$syscfgCli = "D:\ti\ccs2051\sysconfig_1.26.2\sysconfig_cli.bat"
$compilerDir = "D:\ti\ccs2051\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS"
$compiler = Join-Path $compilerDir "bin\tiarmclang.exe"

foreach ($path in @($sdkDir, $syscfgCli, $compiler)) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Required build dependency not found: $path"
    }
}

New-Item -ItemType Directory -Force -Path $objDir, $syscfgDir | Out-Null

& $syscfgCli `
    -s (Join-Path $sdkDir ".metadata\product.json") `
    --script (Join-Path $projectDir "empty_cpp.syscfg") `
    -o $syscfgDir `
    --compiler ticlang
if ($LASTEXITCODE -ne 0) {
    throw "SysConfig generation failed with exit code $LASTEXITCODE"
}

$linkerCmd = Join-Path $syscfgDir "device_linker.cmd"
if (Test-Path $linkerCmd) {
    $content = Get-Content $linkerCmd -Raw
    if ($content -match '--stack_size=512') {
        $content -replace '--stack_size=512', '--stack_size=1024' | Set-Content $linkerCmd -NoNewline
    }
}

$includeArgs = @(
    "-I$projectDir",
    "-I$syscfgDir",
    "-I$(Join-Path $sdkDir 'source')",
    "-I$(Join-Path $sdkDir 'source\third_party\CMSIS\Core\Include')"
)

# 包含 modules 目录
Get-ChildItem (Join-Path $projectDir "modules") -Directory -Recurse -ErrorAction SilentlyContinue | ForEach-Object {
    $includeArgs += "-I$($_.FullName)"
}

$compileArgs = @(
    "-c",
    "-march=thumbv6m",
    "-mcpu=cortex-m0plus",
    "-mfloat-abi=soft",
    "-mlittle-endian",
    "-mthumb",
    "-O2",
    "-gdwarf-3",
    "-D__MSPM0G3507__",
    "-D__USE_SYSCONFIG__"
) + $includeArgs

# 查找所有 .c/.cpp 源文件
$sources = @()
Get-ChildItem $projectDir -Include "*.c", "*.cpp" -Recurse -File -ErrorAction SilentlyContinue | Where-Object {
    $_.FullName -notmatch '\\Debug\\' -and
    $_.FullName -notmatch '\\.ccsproject' -and
    $_.FullName -notmatch '\\.settings\\'
} | ForEach-Object {
    $relPath = $_.FullName.Substring($projectDir.Length + 1)
    $objName = $_.BaseName + ".o"
    $sources += @{ Source = $_.FullName; Object = $objName }
}

# 添加 SysConfig 生成的文件
$sources += @{
    Source = (Join-Path $syscfgDir "ti_msp_dl_config.c")
    Object = "ti_msp_dl_config.o"
}
$sources += @{
    Source = (Join-Path $sdkDir "source\ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c")
    Object = "startup_mspm0g350x_ticlang.o"
}

$objects = foreach ($item in $sources) {
    $source = $item.Source
    $object = Join-Path $objDir $item.Object
    & $compiler @compileArgs -o $object $source
    if ($LASTEXITCODE -ne 0) {
        throw "Compilation failed: $source"
    }
    $object
}

$output = Join-Path $buildDir "empty_cpp.out"
$map = Join-Path $buildDir "empty_cpp.map"
$linkArgs = @(
    "-march=thumbv6m",
    "-mcpu=cortex-m0plus",
    "-mfloat-abi=soft",
    "-mlittle-endian",
    "-mthumb",
    "-O2",
    "-gdwarf-3",
    "-Wl,-m$map",
    "-Wl,-i$(Join-Path $sdkDir 'source')",
    "-Wl,-i$syscfgDir",
    "-Wl,-i$(Join-Path $compilerDir 'lib')",
    "-Wl,--diag_wrap=off",
    "-Wl,--display_error_number",
    "-Wl,--warn_sections",
    "-Wl,--rom_model",
    "-o",
    $output
) + $objects + @(
    "-Wl,-ldevice_linker.cmd",
    "-Wl,-ldevice.cmd.genlibs",
    "-Wl,-llibc.a"
)

& $compiler @linkArgs
if ($LASTEXITCODE -ne 0) {
    throw "Link failed with exit code $LASTEXITCODE"
}

Write-Host "Build completed: $output"
