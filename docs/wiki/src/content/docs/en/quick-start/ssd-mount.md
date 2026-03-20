---
title: 4.5 SSD Mount
description: SSD Partition Initialization and Mounting
sidebar:
  order: 5
---

An NVMe SSD is pre-installed in the onboard computer. Run the following commands in the terminal to mount the SSD.

```bash
# SSD partition initialization
sudo /sbin/parted /dev/nvme0n1 mklabel gpt --script
sudo mkfs.ext4 /dev/nvme0n1

# SSD mount
sudo mount /dev/nvme0n1 {mount folder path}
```
