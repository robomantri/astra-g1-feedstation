#!/usr/bin/env bash
export DEBIAN_FRONTEND=noninteractive
{
  echo "=== rollback record ==="
  dpkg -l | awk "/^ii/ && /nvidia/ {print \$2\"=\"\$3}" > /root/nvidia-535-packages.txt
  echo ">>> installing nvidia-driver-570"
  apt-get install -y nvidia-driver-570 2>&1 | tail -25
  echo ">>> fixing any half-configured"
  apt-get -f install -y 2>&1 | tail -8
  dpkg --configure -a 2>&1 | tail -8
  echo ">>> force-overwrite any blocked libs (the 610->535 lesson)"
  for d in /var/cache/apt/archives/libnvidia-*570*.deb; do
    [ -f "$d" ] && dpkg -i --force-overwrite "$d" 2>&1 | grep -aiE "^dpkg: error|overwrite" | head -2
  done
  apt-get -f install -y 2>&1 | tail -5
  echo "=== DKMS ==="; dkms status
  echo "=== package states ==="; dpkg -l | grep -E "nvidia.*570" | awk "{print \$1, \$2}"
  echo "=== leftover 535 ==="; dpkg -l | awk "/^ii/ && /535\./ {print \$2}" | head
  echo DRV570_DONE
} > /root/drv570.log 2>&1
