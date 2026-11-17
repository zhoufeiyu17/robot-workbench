#!/bin/bash
set -euo pipefail
umask 077
python=${1:?registered Python interpreter required}
config=${2:?registered desktop configuration required}
mode=${3:-}
[[ $# -le 3 && ( -z "$mode" || "$mode" == '--no-browser' ) ]] || exit 2
source_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
log_root="${config}.startup"
mkdir -p -- "$log_root"
result=$(mktemp "$log_root/desktop-start-XXXXXXXX.json")
page_opened=false
failed() {
  if [[ "$mode" != '--no-browser' ]]; then
    title='真机工作台页面打开失败'
    severity=--error
    if $page_opened; then
      title='页面已打开，设备准备未完成'
      severity=--warning
    fi
    zenity "$severity" --title="$title" --text="$(tail -n 1 "$result.stderr")
启动记录：$result.stderr" || true
  fi
  cat "$result.stderr" >&2
  exit 1
}
if [[ "$mode" != '--no-browser' ]]; then
  if ! "$python" "$source_root/scripts/start_registered_workbench.py" --config "$config" --open-page-only 2>"$result.stderr"; then failed; fi
  page_opened=true
fi
exec 9>"${config}.desktop.lock"
if ! flock -n 9; then
  printf '%s\n' '{"status":"preparation_in_progress"}'
  exit 0
fi
if ! "$python" "$source_root/scripts/start_registered_workbench.py" --config "$config" --takeover >"$result" 2>>"$result.stderr"; then failed; fi
cat "$result"
