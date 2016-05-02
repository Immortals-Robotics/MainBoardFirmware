#include "tsk3000.lsl"

derivative system
{
    core sw
    {
        architecture = __SW_ARCH;
    }

    memory irom
    {
        mau = 0x00000008;
        type = rom;
        size = 16k;
        map(dest=bus:sw:addr_bus, src_offset=0x0, dest_offset=0x00000000, size=16k);
    }

    memory iram
    {
        mau = 8;
        type = ram;
        size = 16k;
        map(dest=bus:sw:addr_bus, src_offset=0x0, dest_offset=16k, size=16k);
    }












}
