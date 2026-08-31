param(
    [Parameter(Mandatory = $true)]
    [string]$Binary
)

$bytes = [IO.File]::ReadAllBytes($Binary)
if ($bytes.Length -lt 4 -or ($bytes.Length % 4) -ne 0) {
    throw "Class-B image length must be a non-zero multiple of four bytes."
}

[uint64]$crc = 4294967295
$bytesToCheck = $bytes.Length - 4

for ($offset = 0; $offset -lt $bytesToCheck; $offset += 4) {
    # This matches CRC->DR = __REV(*(uint32_t *)address) in
    # stm32fxx_STLstartup.c. The target is little-endian, so construct the
    # value after __REV directly from the four Flash bytes.
    [uint64]$word = (([uint64]$bytes[$offset] -shl 24) -bor
                     ([uint64]$bytes[$offset + 1] -shl 16) -bor
                     ([uint64]$bytes[$offset + 2] -shl 8) -bor
                     [uint64]$bytes[$offset + 3])
    $crc = ($crc -bxor $word) -band 0xFFFFFFFFL

    for ($bit = 0; $bit -lt 32; $bit++) {
        if (($crc -band 0x80000000L) -ne 0) {
            $crc = (($crc -shl 1) -bxor 0x04C11DB7L) -band 0xFFFFFFFFL
        }
        else {
            $crc = ($crc -shl 1) -band 0xFFFFFFFFL
        }
    }
}

$reference = [BitConverter]::ToUInt32($bytes, $bytesToCheck)
$expectedText = '0x{0:X8}' -f $crc
$referenceText = '0x{0:X8}' -f $reference

if ($reference -ne [uint32]$crc) {
    throw "Class-B checksum mismatch: startup contains $referenceText, but the firmware computes $expectedText. Update _Check_Sum in cmake/startup_stm32f091xc.s and rebuild."
}

Write-Host "Class-B reference CRC verified: $referenceText"
