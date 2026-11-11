/*
 * Minimal, deterministic FAT32 volume builder - implementation.
 *
 * SPDX-License-Identifier: MIT
 */

#include "fat32_builder.h"

#include <string.h>

#define FAT32_MIN_CLUSTERS 65525u // below this FAT32 is not a legal choice
#define FAT32_MEDIA_DESC 0xF8u
#define FAT32_EOC 0x0FFFFFFFu

static inline void put16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static inline void put32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

// Legacy cylinder/head/sector triple for the partition table. Only used by
// ancient tools; 255 heads / 63 sectors is the conventional fake geometry.
static void put_chs(uint8_t *p, uint32_t lba) {
  uint32_t cyl = lba / (255u * 63u);
  uint32_t head = (lba / 63u) % 255u;
  uint32_t sector = (lba % 63u) + 1u;

  if (cyl > 1023u) {
    cyl = 1023u;
    head = 254u;
    sector = 63u;
  }

  p[0] = (uint8_t)head;
  p[1] = (uint8_t)((sector & 0x3Fu) | ((cyl >> 2) & 0xC0u));
  p[2] = (uint8_t)(cyl & 0xFFu);
}

bool fat32_plan(uint32_t device_sectors, uint32_t sector_size,
                uint32_t volume_id, const char *label, fat32_layout_t *out) {
  if (out == NULL || sector_size != 512u) {
    return false; // only 512-byte logical sectors are supported
  }
  if (device_sectors <= FAT32_PART_START_LBA + FAT32_MIN_CLUSTERS) {
    return false;
  }

  memset(out, 0, sizeof(*out));
  out->sector_size = sector_size;
  out->part_start_lba = FAT32_PART_START_LBA;
  out->part_sectors = device_sectors - FAT32_PART_START_LBA;
  out->reserved_sectors = 128; // room for the boot sector, FSInfo, backup pair
  out->num_fats = 1;           // a second FAT doubles a potentially 1 MiB rewrite
  out->root_cluster = 2;
  out->volume_id = volume_id;

  // Bigger clusters => far fewer FAT entries to zero. 64 KiB keeps the FAT for
  // a 16..32 GiB stick around 1 MiB, which is what makes the rebuild quick.
  const uint64_t bytes = (uint64_t)device_sectors * sector_size;
  if (bytes >= (8ull << 30)) {
    out->sectors_per_cluster = 128; // 64 KiB
  } else if (bytes >= (2ull << 30)) {
    out->sectors_per_cluster = 64; // 32 KiB
  } else {
    out->sectors_per_cluster = 8; // 4 KiB
  }

  // Solve for the FAT size: it depends on the cluster count, which depends on
  // the FAT size. Converges in a couple of rounds.
  uint32_t fat_sectors = 1;
  for (int i = 0; i < 16; i++) {
    const uint32_t usable =
        out->part_sectors - out->reserved_sectors - out->num_fats * fat_sectors;
    if (usable <= out->sectors_per_cluster) {
      return false;
    }
    const uint32_t clusters = usable / out->sectors_per_cluster;
    const uint32_t need =
        ((clusters + 2u) * 4u + sector_size - 1u) / sector_size;
    if (need <= fat_sectors) {
      break;
    }
    fat_sectors = need;
  }

  out->fat_sectors = fat_sectors;
  out->cluster_count =
      (out->part_sectors - out->reserved_sectors - out->num_fats * fat_sectors) /
      out->sectors_per_cluster;

  if (out->cluster_count < FAT32_MIN_CLUSTERS) {
    return false; // too few clusters for a legal FAT32 volume
  }

  memset(out->label, ' ', 11);
  out->label[11] = '\0';
  if (label != NULL) {
    size_t n = strlen(label);
    if (n > 11) {
      n = 11;
    }
    memcpy(out->label, label, n);
  }

  return true;
}

uint32_t fat32_region_sectors(const fat32_layout_t *layout) {
  return layout->reserved_sectors +
         layout->num_fats * layout->fat_sectors +
         layout->sectors_per_cluster; // root directory occupies one cluster
}

static void build_boot_sector(const fat32_layout_t *layout, uint8_t *s) {
  s[0] = 0xEB; // jmp short 0x58 ; nop
  s[1] = 0x58;
  s[2] = 0x90;
  memcpy(s + 3, "MSWIN4.1", 8); // OEM name

  put16(s + 11, (uint16_t)layout->sector_size);
  s[13] = (uint8_t)layout->sectors_per_cluster;
  put16(s + 14, (uint16_t)layout->reserved_sectors);
  s[16] = (uint8_t)layout->num_fats;
  put16(s + 17, 0);  // root directory entries (0 for FAT32)
  put16(s + 19, 0);  // 16-bit total sectors
  s[21] = FAT32_MEDIA_DESC;
  put16(s + 22, 0);  // 16-bit FAT size
  put16(s + 24, 63); // sectors per track (fake geometry)
  put16(s + 26, 255);
  put32(s + 28, layout->part_start_lba); // hidden sectors
  put32(s + 32, layout->part_sectors);   // 32-bit total sectors

  // FAT32 extended BPB
  put32(s + 36, layout->fat_sectors);
  put16(s + 40, 0); // extended flags: mirroring enabled
  put16(s + 42, 0); // file system version 0.0
  put32(s + 44, layout->root_cluster);
  put16(s + 48, 1); // FSInfo sector (relative to partition start)
  put16(s + 50, 6); // backup boot sector

  s[64] = 0x80;               // drive number
  s[65] = 0x00;               // reserved
  s[66] = 0x29;               // extended boot signature
  put32(s + 67, layout->volume_id);
  memcpy(s + 71, layout->label, 11);
  memcpy(s + 82, "FAT32   ", 8);

  s[510] = 0x55;
  s[511] = 0xAA;
}

static void build_fs_info(const fat32_layout_t *layout, uint8_t *s) {
  put32(s + 0, 0x41615252u);   // lead signature
  put32(s + 484, 0x61417272u); // structure signature
  put32(s + 488, layout->cluster_count - 1u); // one cluster in use (root dir)
  put32(s + 492, 0xFFFFFFFFu);                // next free cluster unknown
  put32(s + 508, 0xAA550000u);                // trail signature
}

void fat32_fill_sector(const fat32_layout_t *layout, uint32_t abs_lba,
                       uint8_t *sector) {
  memset(sector, 0, layout->sector_size);

  if (abs_lba < layout->part_start_lba) {
    return;
  }
  const uint32_t rel = abs_lba - layout->part_start_lba;

  if (rel == 0 || rel == 6) {
    build_boot_sector(layout, sector);
    return;
  }
  if (rel == 1 || rel == 7) {
    build_fs_info(layout, sector);
    return;
  }

  // FAT area: only the first sector of each copy carries entries; everything
  // else must be zero (free).
  const uint32_t fat_start = layout->reserved_sectors;
  const uint32_t fat_total = layout->fat_sectors * layout->num_fats;
  if (rel >= fat_start && rel < fat_start + fat_total) {
    if ((rel - fat_start) % layout->fat_sectors == 0) {
      put32(sector + 0, 0x0FFFFFF8u); // media descriptor + reserved
      put32(sector + 4, FAT32_EOC);   // clean shutdown, no errors
      put32(sector + 8, 0x0FFFFFF8u); // cluster 2: end of root dir chain
    }
    return;
  }

  // First sector of the root directory cluster: one volume-label entry, then
  // the 0x00 end-of-directory marker (the rest of the cluster stays zero).
  const uint32_t root_start = fat_start + fat_total;
  if (rel == root_start) {
    memcpy(sector, layout->label, 11); // entry name is the label itself
    sector[11] = 0x08;                // ATTR_VOLUME_ID
    // 0x0C..0x1F stay zero: no timestamps, cluster or size for a label.
  }
}

void fat32_fill_mbr(const fat32_layout_t *layout, uint8_t *sector) {
  memset(sector, 0, 512);

  uint8_t *p = sector + 446; // first partition entry
  p[0] = 0x00;               // not bootable
  put_chs(p + 1, layout->part_start_lba);
  p[4] = 0x0C; // FAT32 LBA
  put_chs(p + 5, layout->part_start_lba + layout->part_sectors - 1u);
  put32(p + 8, layout->part_start_lba);
  put32(p + 12, layout->part_sectors);

  sector[510] = 0x55;
  sector[511] = 0xAA;
}
