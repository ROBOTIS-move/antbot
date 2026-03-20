---
title: 4.5 SSD 마운트
description: SSD 파티션 초기화 및 마운트
sidebar:
  order: 5
---

온보드 컴퓨터에는 NVMe SSD가 기본 장착되어 있습니다. SSD를 마운트하기 위해 아래 명령어를 터미널에서 실행합니다.

```bash
# SSD 파티션 초기화
sudo /sbin/parted /dev/nvme0n1 mklabel gpt --script
sudo mkfs.ext4 /dev/nvme0n1

# SSD 마운트
sudo mount /dev/nvme0n1 {마운트 폴더 경로}
```
