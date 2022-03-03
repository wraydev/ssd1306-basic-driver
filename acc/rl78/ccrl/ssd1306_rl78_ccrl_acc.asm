;	ssd1306_rl78_llvm_acc.asm
;	@brief assembly code for CLANG compiler on RL78 to accelerate RAM drawing.
;	@details
;	Contains the following functions:
;		- void Set_pixel(uint16_t x, uint16_t y);
;		- void Clear_pixel(uint16_t x, uint16_t y);

; Global variables required by file (buffer)
.EXTERN _ssd1306_ctl

.PUBLIC	_Set_pixel
.PUBLIC _Clear_pixel

; Defines log2 of buffer width.
DISPLAY_PIXEL_LENGTH_LOG2	.EQU	7

; bit setting lookup table
	.DSEG	DATA
_bset_lookup_table:	.DB	0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80

; bit clearing lookup table
	.DSEG	DATA
_bclear_lookup_table:	.DB	0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F

; START OF FUNCTION - Set_pixel(uint16_t x, uint16_t y) = 18clks + call(3) + ret(6) = 28
; AX = x
; BC = y
; Uses 2bytes of stack
.text	.CSEG   TEXT
_Set_pixel:
	XCHW AX, BC	; 1clk - Swap x and y --> AX=y, BC=x
	PUSH AX		; 1clk - Save y
	XCH	A, X	; 1clk
	AND  A, #7	; 1clk - Compute y_bit
	XCH	A, X	; 1clk
	MOVW DE, AX	; 1clk - Save in DE
	POP  AX		; 1clk - Restore original value of y
	SHRW AX, 3	; 1clk - Compute y_row
	SHLW AX, DISPLAY_PIXEL_LENGTH_LOG2 ; 1clk
	ADDW AX, BC	; 1clk - Compute buffer offset (x + y_row)
	ADDW AX, #LOWW _ssd1306_ctl	; 1clk - Move to correct offset of buffer address
	PUSH AX	; 1clk - save address of byte to edit
	MOVW AX, #LOWW _bset_lookup_table ; 1clk - get lookup table
	ADDW AX, DE	; 1clk - move to lookup offset
	MOVW HL, AX ; 1clk - move data into ax
	MOV A, [HL] ; 1clk
	POP HL ; 1clk
	OR	 A, [HL]; 1clk - set the bit
	MOV  [HL],A ; 1clk - move the data into the buffer.
	RET
; END OF FUNCTION - Set_pixel(uint16_t x, uint16_t y)

; START OF FUNCTION - Clear_pixel(uint16_t x, uint16_t y) = 18clks + call(3) + ret(6) = 28
; AX = x
; BC = y
; Uses 2bytes of stack
.text	.CSEG   TEXT
_Clear_pixel:
	XCHW AX, BC	; 1clk - Swap x and y --> AX=y, BC=x
	PUSH AX		; 1clk - Save y
	XCH	A, X	; 1clk
	AND  A, #7	; 1clk - Compute y_bit
	XCH	A, X	; 1clk
	MOVW DE, AX	; 1clk - Save in DE
	POP  AX		; 1clk - Restore original value of y
	SHRW AX, 3	; 1clk - Compute y_row
	SHLW AX, DISPLAY_PIXEL_LENGTH_LOG2 ; 1clk
	ADDW AX, BC	; 1clk - Compute buffer offset (x + y_row)
	ADDW AX, #LOWW _ssd1306_ctl	; 1clk - Move to correct offset of buffer address
	PUSH AX	; 1clk - save address of byte to edit
	MOVW AX, #LOWW _bclear_lookup_table ; 1clk - get lookup table
	ADDW AX, DE	; 1clk - move to lookup offset
	MOVW HL, AX ; 1clk - move data into ax
	MOV A, [HL] ; 1clk
	POP HL ; 1clk
	AND	 A, [HL]; 1clk - clear the bit
	MOV  [HL],A ; 1clk - move it into buffer
	RET
; END OF FUNCTION - Clear_pixel(uint16_t x, uint16_t y)

