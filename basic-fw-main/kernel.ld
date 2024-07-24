OUTPUT_FORMAT("elf64-littleaarch64")
OUTPUT_ARCH(aarch64)
/*TARGET(binary)*/
ENTRY(__start)

SECTIONS
{
	. = 0x48000000;
	_bfw_start_point = .;
	.entry : {*(.entry) }
	.text : { *(.text) }
	.data : { *(.data) }
	. = ALIGN(8);
	.bss (NOLOAD): 
	{ 
		_bss_start = .;
		PROVIDE(BSS_START_ADDR = .);
		*(.bss)
		. = ALIGN(8);
		_bss_end = .; 
		PROVIDE(BSS_END_ADDR = .);
	}

	. = ALIGN(0x1000);
	stacktop = .;	
}
