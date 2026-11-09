/* HT32F1654: reserve all flash from 0x2000 onward for the application.
 * The final 1KiB page at 0xFC00 is outside main flash and must not be used.
 */
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 8K
  RAM   : ORIGIN = 0x20000000, LENGTH = 16K
}

ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(FLASH) + LENGTH(FLASH),
       "bootloader exceeds its exact 8KiB flash region");
