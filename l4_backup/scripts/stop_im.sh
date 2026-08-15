#!/usr/bin/env bash
# kill by explicit PID; avoid pkill -f self-match
for p in $(pgrep -f "python -m intermimic"); do
  if [ "$p" != "$$" ] && [ "$p" != "$PPID" ]; then
    kill "$p" 2>/dev/null && echo "killed $p"
  fi
done
