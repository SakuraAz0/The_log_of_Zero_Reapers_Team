<#
  把本工程（CRTC_ZRTeam_ROBORT1）当前提交的快照上传到 GitHub 团队的 Source_Code/ 目录。

  远端布局：
    Source_Code/CRTC_ZRTeam_ROBORT1/   工程本体（来自本仓库 HEAD）
    Source_Code/docs/                  本仓库 docs/ 下的文档
    （团队原有的 README.md、Source_Code/log、docs/README.md 都不动）

  只上传 git 已跟踪的文件（不含 .idea、cmake-build-* 等构建产物）。
  每次运行产生一个提交；本仓库自己的提交历史与目录结构不受影响。

  用法（在仓库根目录执行，Windows PowerShell 5.1 或 PowerShell 7 均可）：
    powershell -NoProfile -File tools/upload_to_github.ps1
    powershell -NoProfile -File tools/upload_to_github.ps1 -Message "更新遥控映射：左X改为自转"
  若已装 PowerShell 7，把 powershell 换成 pwsh 亦可。
#>
param(
    [string]$Message  = "",
    [string]$RemoteUrl = 'https://github.com/SakuraAz0/The_log_of_Zero_Reapers_Team',
    [string]$Branch    = 'main'
)

$ErrorActionPreference = 'Stop'
$env:GIT_TERMINAL_PROMPT = '0'      # 不要让 git 在非交互环境下卡住

$root = Split-Path -Parent $PSScriptRoot          # 仓库根目录
$tmp  = Join-Path $root '_gh_upload'              # 临时克隆（已在 .gitignore 中）
$tar  = Join-Path $root '_gh_src.tar'

if (-not $Message) {
    $head = (git -C $root log -1 --pretty=%s)
    $Message = "Update CRTC_ZRTeam_ROBORT1 source code ($head)"
}

Write-Host "[1/5] 导出已跟踪文件：git archive HEAD"
if (Test-Path $tar) { Remove-Item $tar -Force }
git -C $root archive --format=tar -o $tar HEAD
if ($LASTEXITCODE -ne 0) { throw "git archive 失败" }

Write-Host "[2/5] 准备远端克隆：$tmp"
if (Test-Path (Join-Path $tmp '.git')) {
    git -C $tmp fetch -q origin $Branch
    git -C $tmp checkout -q -B $Branch "origin/$Branch"
} else {
    if (Test-Path $tmp) { Remove-Item $tmp -Recurse -Force }
    git clone -q $RemoteUrl $tmp
    git -C $tmp checkout -q $Branch
}
if ($LASTEXITCODE -ne 0) { throw "克隆/更新远端失败" }

Write-Host "[3/5] 替换 Source_Code/CRTC_ZRTeam_ROBORT1 与 Source_Code/docs"
$dstProj = Join-Path $tmp 'Source_Code\CRTC_ZRTeam_ROBORT1'
$dstDocs = Join-Path $tmp 'Source_Code\docs'
foreach ($d in @($dstProj, $dstDocs)) {
    if (Test-Path $d) { Remove-Item $d -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $d | Out-Null
}

# 解压：本工程的源文件名都是 ASCII，bsdtar 没问题
tar -xf $tar -C $dstProj

# 文档用 PowerShell 原生复制（bsdtar 在 Windows 上处理中文文件名会报 "Invalid empty pathname"）
$srcDocs = Join-Path $root 'docs'
if (Test-Path $srcDocs) {
    Get-ChildItem -LiteralPath $srcDocs -File | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $dstDocs -Force
    }
}
# 文档统一放 Source_Code/docs，工程目录里不重复
$projDocs = Join-Path $dstProj 'docs'
if (Test-Path $projDocs) { Remove-Item $projDocs -Recurse -Force }

Write-Host "[4/5] 提交"
git -C $tmp add -A
$changes = (git -C $tmp status --short | Measure-Object).Count
if ($changes -eq 0) {
    Write-Host "远端已是最新，无需提交"
} else {
    git -C $tmp commit -q -m $Message
    if ($LASTEXITCODE -ne 0) { throw "提交失败" }
    Write-Host "[5/5] 推送到 origin/$Branch"
    git -C $tmp push origin $Branch
    if ($LASTEXITCODE -ne 0) { throw "推送失败（检查 GitHub 凭据/权限）" }
    Write-Host ("完成：{0} 个文件变更已推送" -f $changes)
}

Remove-Item $tar -Force -ErrorAction SilentlyContinue
Write-Host "远端地址：$RemoteUrl"
