/*
 * Host-side sanity generator for fat32_builder.
 *
 * Builds two sparse image files from the same code the firmware uses:
 *   disk.img - full device with MBR + FAT32 volume at LBA 2048
 *   part.img - just the volume, so fsck.fat can check it directly
 *
 * Usage: fat32_image <device_sectors> <out_prefix>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fat32_builder.h"

static int write_at(FILE *f, long offset, const void *buf, size_t len) {
  if (fseek(f, offset, SEEK_SET) != 0) {
    return -1;
  }
  return fwrite(buf, 1, len, f) == len ? 0 : -1;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "usage: %s <device_sectors> <out_prefix>\n", argv[0]);
    return 2;
  }

  const uint32_t device_sectors = (uint32_t)strtoul(argv[1], NULL, 0);
  char disk_path[512];
  char part_path[512];
  snprintf(disk_path, sizeof(disk_path), "%s_disk.img", argv[2]);
  snprintf(part_path, sizeof(part_path), "%s_part.img", argv[2]);

  fat32_layout_t layout;
  if (!fat32_plan(device_sectors, 512, 0x5049434Fu, "CLEANED", &layout)) {
    fprintf(stderr, "fat32_plan() rejected %u sectors\n", device_sectors);
    return 1;
  }

  const uint32_t region = fat32_region_sectors(&layout);
  printf("device_sectors=%u part_start=%u part_sectors=%u\n", device_sectors,
         layout.part_start_lba, layout.part_sectors);
  printf("spc=%u reserved=%u fat_sectors=%u clusters=%u region_sectors=%u "
         "(%.1f KiB to write)\n",
         layout.sectors_per_cluster, layout.reserved_sectors,
         layout.fat_sectors, layout.cluster_count, region,
         (double)region * 512.0 / 1024.0);

  uint8_t sector[512];

  FILE *disk = fopen(disk_path, "wb");
  FILE *part = fopen(part_path, "wb");
  if (!disk || !part) {
    perror("fopen");
    return 1;
  }

  if (fseek(disk, (long)device_sectors * 512L - 1, SEEK_SET) != 0 ||
      fputc(0, disk) == EOF) {
    perror("truncate disk");
    return 1;
  }
  if (fseek(part, (long)layout.part_sectors * 512L - 1, SEEK_SET) != 0 ||
      fputc(0, part) == EOF) {
    perror("truncate part");
    return 1;
  }

  fat32_fill_mbr(&layout, sector);
  if (write_at(disk, 0, sector, sizeof(sector)) != 0) {
    perror("write mbr");
    return 1;
  }

  for (uint32_t i = 0; i < region; i++) {
    const uint32_t abs_lba = layout.part_start_lba + i;
    fat32_fill_sector(&layout, abs_lba, sector);
    if (write_at(disk, (long)abs_lba * 512L, sector, sizeof(sector)) != 0 ||
        write_at(part, (long)i * 512L, sector, sizeof(sector)) != 0) {
      perror("write region");
      return 1;
    }
  }

  fclose(disk);
  fclose(part);
  printf("wrote %s and %s\n", disk_path, part_path);
  return 0;
}
