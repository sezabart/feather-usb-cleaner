/*
 * Minimal, deterministic FAT32 volume builder.
 *
 * Produces the bytes for an empty MBR-partitioned FAT32 file system without
 * pulling in a file system library: an MBR, a FAT32 boot sector + FSInfo +
 * backup boot sectors, one FAT, and an empty root directory cluster.
 *
 * Deliberately dependency-free (no Pico SDK, no libc beyond <string.h>) so the
 * same code can be exercised on a PC and checked with fsck.fat.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef FAT32_BUILDER_H
#define FAT32_BUILDER_H

#include <stdbool.h>
#include <stdint.h>

// 1 MiB alignment, the conventional start of the first partition.
#define FAT32_PART_START_LBA 2048u

typedef struct {
  uint32_t sector_size;         // bytes per logical sector (512)
  uint32_t sectors_per_cluster; // 8..128 (4 KiB .. 64 KiB)
  uint32_t reserved_sectors;
  uint32_t num_fats;            // 1 keeps the FAT rewrite small
  uint32_t fat_sectors;         // sectors per FAT
  uint32_t part_start_lba;
  uint32_t part_sectors;
  uint32_t cluster_count;
  uint32_t root_cluster;        // always 2
  uint32_t volume_id;
  char label[12];               // 11 chars + NUL
} fat32_layout_t;

// Work out the geometry for a device of `device_sectors` logical sectors.
// Fails if the device is too small for a legal FAT32 (cluster count < 65525).
bool fat32_plan(uint32_t device_sectors, uint32_t sector_size,
                uint32_t volume_id, const char *label, fat32_layout_t *out);

// Number of sectors from the partition start that must be written to make the
// volume valid and empty (reserved area + FATs + root directory cluster).
uint32_t fat32_region_sectors(const fat32_layout_t *layout);

// Fill one 512-byte (or sector_size-byte) sector at absolute device LBA
// `abs_lba`, which must lie in [part_start_lba, part_start_lba + region).
void fat32_fill_sector(const fat32_layout_t *layout, uint32_t abs_lba,
                       uint8_t *sector);

// Fill the MBR sector (absolute LBA 0).
void fat32_fill_mbr(const fat32_layout_t *layout, uint8_t *sector);

#endif // FAT32_BUILDER_H
