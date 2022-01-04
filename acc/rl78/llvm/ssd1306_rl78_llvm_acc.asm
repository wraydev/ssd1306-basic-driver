;	ssd1306_rl78_llvm_acc.asm
;	@brief assembly code for CLANG compiler on RL78 to accelerate RAM drawing.
;	@details
;	Contains the following functions:
;		- void Set_pixel(uint16_t x, uint16_t y);
;		- void Clear_pixel(uint16_t x, uint16_t y);

; Global variables required by file (buffer)
.extern _ssd1306_ctl

; Defines log2 of buffer width.
.equ	DISPLAY_PIXEL_LENGTH_LOG2, 7

; bit setting lookup table
	.data
_bset_lookup_table:
	.byte     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80

; bit clearing lookup table
	.data
_bclear_lookup_table:
	.byte     0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F

; START OF FUNCTION - Set_pixel(uint16_t x, uint16_t y) = 18clks + call(3) + ret(6) = 27
; AX = x
; BC = y
; Uses 2bytes of stack
	.global _Set_pixel
	.type	_Set_pixel,STT_FUNC
	.text
_Set_pixel:
	XCHW AX, BC	; 1clk - Swap x and y --> AX=y, BC=x
	PUSH AX		; 1clk - Save y
	AND  X, #7	; 1clk - Compute y_bit
	MOVW DE, #0	; 1clk
	MOVW DE, AX	; 1clk - Save in DE
	POP  AX		; 1clk - Restore original value of y
	SHRW AX, #3	; 1clk - Compute y_row
	SHLW AX, DISPLAY_PIXEL_LENGTH_LOG2 ; 1clk
	ADDW AX, BC	; 1clk - Compute buffer offset (x + y_row)
	ADDW AX, #_ssd1306_ctl	; 1clk - Move to correct offset of buffer address
	PUSH AX	; 1clk - save address of byte to edit
	MOVW AX, #_bset_lookup_table ; 1clk - get lookup table
	ADDW AX, DE	; 1clk - move to lookup offset
	MOVW HL, AX ; 1clk - move data into ax
	MOV A, [HL] ; 1clk
	POP HL ; 1clk
	OR	 A, [HL]; 1clk - set the bit
	MOV  [HL],A ; 1clk - move the data into the buffer.
	RET
; END OF FUNCTION - Set_pixel(uint16_t x, uint16_t y)

; START OF FUNCTION - Clear_pixel(uint16_t x, uint16_t y) = 18clks + call(3) + ret(6) = 27
; AX = x
; BC = y
; Uses 2bytes of stack
	.global _Clear_pixel
	.type	_Clear_pixel,STT_FUNC
	.text
_Clear_pixel:
	XCHW AX, BC	; 1clk - Swap x and y --> AX=y, BC=x
	PUSH AX		; 1clk - Save y
	AND  X, #7	; 1clk - Compute y_bit
	MOVW DE, #0	; 1clk
	MOVW DE, AX	; 1clk - Save in DE
	POP  AX		; 1clk - Restore original value of y
	SHRW AX, #3	; 1clk - Compute y_row
	SHLW AX, DISPLAY_PIXEL_LENGTH_LOG2 ; 1clk
	ADDW AX, BC	; 1clk - Compute buffer offset (x + y_row)
	ADDW AX, #_ssd1306_ctl	; 1clk - Move to correct offset of buffer address
	PUSH AX	; 1clk - save address of byte to edit
	MOVW AX, #_bclear_lookup_table ; 1clk - get lookup table
	ADDW AX, DE	; 1clk - move to lookup offset
	MOVW HL, AX ; 1clk - move data into ax
	MOV A, [HL] ; 1clk
	POP HL ; 1clk
	AND	 A, [HL]; 1clk - clear the bit
	MOV  [HL],A ; 1clk - move it into buffer
	RET
; END OF FUNCTION - Clear_pixel(uint16_t x, uint16_t y)

