# Ядро Linux 6.6.18 для линейки принтеров Creality K1 на процессоре Ingenic X2000E

## Компиляция

```bash
cd ~/work/mips-x2000e/linux

export TOOLCHAIN="$HOME/work/mips-x2000e/mips32r5el--glibc--bleeding-edge-2025.08-1"
export PATH="$TOOLCHAIN/bin:$PATH"
export CROSS_COMPILE=mipsel-linux-
export ARCH=mips

make -j$(nproc) ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE
make -j$(nproc) ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE uImage

make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE -j$(nproc) modules
make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE INSTALL_MOD_PATH=~/work/mips-x2000e/rootfs modules_install
```

## Адреса загрузки ядра через uBoot

```bash
loady 0x80f00000
bootm 0x80f00000
```

## Запись модулей

```bash
tar -czf modules-6.6.18+.tar.gz 6.6.18+
sudo tar -xzf modules-6.6.18+.tar.gz -C /lib/modules/

sudo depmod -a $(uname -r)
```

## Бэкап ядра

```bash
sudo dd if=/home/printer/uImage.bin of=/dev/mmcblk0p3
```

## Запись ядра

# Заполняем раздел p3 нулями

```bash
sudo dd if=/dev/zero of=/dev/mmcblk0p3 bs=4K
```

# Записываем новый образ поверх нулей

```bash
sudo dd if=/home/printer/uImage.gz of=/dev/mmcblk0p3
```

# Синхронизация!

```bash
sync
```

## Драйвера wifi

❯ sudo mv /lib/firmware/brcm/brcmfmac43430-sdio.AP6212.clm_blob /lib/firmware/brcm/brcmfmac43430-sdio.clm_blob.bak
