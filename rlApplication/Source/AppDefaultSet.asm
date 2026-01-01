.SECTION DEFAULT_SET,	TEXT	
 DEFAULT_SET .CSEG		TEXT	
.ORG	0x400

$IFNDEF ASM_PCB_VER_4_6_8
ASM_PCB_VER_4_6_8		.EQU	1
$ENDIF
$IFNDEF ASM_PCB_VER_4_6_6
ASM_PCB_VER_4_6_6	.EQU		0
$ENDIF
$IFNDEF ASM_PCB_VER_4_6_2
ASM_PCB_VER_4_6_2	.EQU		0
$ENDIF
$IFNDEF ASM_TARGET_BOARD_HPSEB
ASM_TARGET_BOARD_HPSEB	.EQU	0
$ENDIF
$IFNDEF ASM_TARGET_BOARD_MVVNL
ASM_TARGET_BOARD_MVVNL	.EQU		0
$ENDIF
$IFNDEF ASM_TARGET_BOARD_WRANGAL
ASM_TARGET_BOARD_WRANGAL	.EQU	1
$ENDIF


$IF (ASM_PCB_VER_4_6_8==1)
FLASH_FW_VERSION:				.DB "4.68.308"
$ENDIF
$IF (ASM_PCB_VER_4_6_6==1)
FLASH_FW_VERSION:				.DB "4.66.621"
$ENDIF
$IF (ASM_PCB_VER_4_6_2==1)
FLASH_FW_VERSION:				.DB "4.62.304"
$ENDIF
								.DS	0x08
FLASH_NO_OF_CATEGORY:			.DB 0x11
$IF (ASM_TARGET_BOARD_MVVNL==1)
; parameters' catogery defined in following format:1- byte-no of bytes, values
; lower bytes first
;#1
FLASH_PRE_BILL_SINGLE_ACTION:		.DB	0x05,0x00,0x00,0x00,0x00,0xFF
;#2
FLASH_PASSIVE_SEA_ACT_TIME:		.DB	0x05,0x00,0x00,0x00,0x00,0xFF
;#3
FLASH_MD_PERIOD:						.DB	0x02,0x08,0x07	;	1800
;#4
FLASH_LS_PERIOD:						.DB 	0x02,0x02,0x00	;	1800
;#5
FLASH_PRE_BILL_DAY_TYPE:					.DB 	0x02			;	size
								.DB	0x01			;   day of bill
								.DB	0x00			;	month of 0-Each,1-Odd,2-Even
;#6								
FLASH_ACTIVE_CAL_NAME:						.DB 	0x36			;	size
								.DB	'A','C','A','L','1',0x00,0x00,0x00		;	Active Calander name
								.DB	'A','S','E','A','S','O','N',0x00		;	Active Season1 name
								.DB	'W','E','E','K','1',0x00,0x00,0x00		;	Active Season1 week name
								.DB	0x01,0x01,0x01,0x01,0x01,0x01,0x01		;	Active Season1 day#
								.DB	'B','S','E','A','S','O','N',0x00		;	Active Season2 name
								.DB	'W','E','E','K','2',0x00,0x00,0x00		;	Active Season2 week name
								.DB	0x02,0x02,0x02,0x02,0x02,0x02,0x02		;	Active Season2 day#
;#7
FLASH_ACTIVE_SEASON1_ZONE_TIME:	.DB 0x19			;	size
								.DB	0x00,0x00		;	TZ1-0
								.DB	0x2C,0x01		;	TZ2-300
								.DB	0x1C,0x02		;	TZ3-540
								.DB	0x38,0x04		;	TZ4-1080
								.DB	0x28,0x05		;	TZ5-1320
								.DB	0x00,0x00		;	TZ6-0
								.DB	0x00,0x00		;	TZ7-0
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x05			; 	Total zones-5
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x03			; 	TZ3-zone no
								.DB	0x04			; 	TZ4-zone no
								.DB	0x01			; 	TZ5-zone no
								.DB	0x00			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#8								
FLASH_ACTIVE_SEASON2_ZONE_TIME:					.DB 	0x19			;	size
								.DB	0x00,0x00		;	TZ1-0
								.DB	0x68,0x01		;	TZ2-360
								.DB	0x58,0x02		;	TZ3-600
								.DB	0xFC,0x03		;	TZ4-1020
								.DB	0xEC,0x04		;	TZ5-1260
								.DB	0x00,0x00		;	TZ6-0
								.DB	0x00,0x00		;	TZ7-0
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x05			; 	Total zones-5
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x03			; 	TZ3-zone no
								.DB	0x04			; 	TZ4-zone no
								.DB	0x01			; 	TZ5-zone no
								.DB	0x00			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#9								
FLASH_ACTIVE_SEASON_TIME:					.DB 	0x07			;	size
								.DB	0x01			; 	season1-start day
								.DB	0x04			; 	season1-start month
								.DB	0x01			; 	season2-start day
								.DB	0x0A			; 	season2-start month
								.DB	0x02			;	total seasons
								.DB	0x01			;	seasons seq-1
								.DB	0x02			;	seasons seq-2
;#10								
FLASH_PASSIVE_CAL_NAME:						.DB 	0x36			;	size
								.DB	'B','C','A','L','1',0x00,0x00,0x00		;	Passive Calander name
								.DB	'A','S','E','A','S','O','N',0x00		;	Passive Season name
								.DB	'W','E','E','K','1',0x00,0x00,0x00		;	Passive week name
								.DB	0x01,0x01,0x01,0x01,0x01,0x01,0x01		;	Passive Season1 day#
								.DB	'B','S','E','A','S','O','N',0x00		;	Passive Season2 name
								.DB	'W','E','E','K','2',0x00,0x00,0x00		;	Passive Season2 week name
								.DB	0x01,0x01,0x01,0x01,0x01,0x01,0x01		;	Passive Season2 day#
;#11								
FLASH_PASSIVE_SEASON1_ZONE_TIME:				.DB 	0x19			;	size
								.DB	0x68,0x01		;	TZ1-360
								.DB	0x38,0x04		;	TZ2-1080
								.DB	0x28,0x05		;	TZ3-1320
								.DB	0x00,0x00		;	TZ4-0
								.DB	0x00,0x00		;	TZ5-0
								.DB	0x00,0x00		;	TZ6-0
								.DB	0x00,0x00		;	TZ7-0
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x03			; 	Total zones-3
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x03			; 	TZ3-zone no
								.DB	0x00			; 	TZ4-zone no
								.DB	0x00			; 	TZ5-zone no
								.DB	0x00			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#12								
FLASH_PASSIVE_SEASON2_ZONE_TIME:				.DB 	0x19			;	size
								.DB	0x00,0x00		;	TZ1-00
								.DB	0x38,0x04		;	TZ2-1080
								.DB	0x00,0x00		;	TZ3-0
								.DB	0x00,0x00		;	TZ4-0
								.DB	0x00,0x00		;	TZ5-0
								.DB	0x00,0x00		;	TZ6-0
								.DB	0x00,0x00		;	TZ7-0
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x02			; 	Total zones-2
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x00			; 	TZ3-zone no
								.DB	0x00			; 	TZ4-zone no
								.DB	0x00			; 	TZ5-zone no
								.DB	0x00			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#13								
FLASH_PASSIVE_SEASON_TIME:					.DB 	0x07			;	size
								.DB	0x01			; 	season1-start day
								.DB	0x04			; 	season1-start month
								.DB	0x01			; 	season2-start day
								.DB	0x0A			; 	season2-start month
								.DB	0x02			;	total seasons
								.DB	0x01			;	seasons seq-1
								.DB	0x02			;	seasons seq-2
;#14
FLASH_IS_MEMORY_CLEARED:					.DB	0x04			;size
								.DB	0x04
								.DB	0x03
								.DB	0x02
								.DB	0x01
;#15
FLASH_DEFAULT_LLS_KEY:						.DB	0x08			;size
								.DB	'A'
								.DB	'B'
								.DB	'C'
								.DB	'D'
								.DB	'E'
								.DB	'F'
								.DB	'G'
								.DB	'H'
;#16
FLASH_DEFAULT_HLS_KEY:						.DB	0x10			;size
								.DB	'R'
								.DB	'E'
								.DB	'N'
								.DB	'E'
								.DB	'S'
								.DB	'A'
								.DB	'S'
								.DB	'_'
								.DB	'P'
								.DB	'6'
								.DB	'w'
								.DB	'R'
								.DB	'J'
								.DB	'2'
								.DB	'1'
								.DB	'F'
;#17
FLASH_DEFAULT_KONO:						.DB	0x0A			;size
								.DB 	'1'
								.DB 	'1'
								.DB 	'1'
								.DB 	'1'
								.DB 	'1'
								.DB 	'1'
								.DB 	'1'
								.DB 	'1'
								.DB 	'1'
								.DB 	'1'
		
							
$ENDIF
$IF (ASM_TARGET_BOARD_HPSEB==1)
; parameters' catogery defined in following format:1- byte-no of bytes, values
; lower bytes first
;#1
FLASH_PRE_BILL_SINGLE_ACTION:	.DB	0x05,0x00,0x00,0x00,0x00,0xFF
;#2
FLASH_PASSIVE_SEA_ACT_TIME:		.DB	0x05,0x00,0x00,0x00,0x00,0xFF
;#3
FLASH_MD_PERIOD:				.DB	0x02,0x08,0x07	;	1800
;#4
FLASH_LS_PERIOD:				.DB 0x02,0x02,0x00	;	1800
;#5
FLASH_PRE_BILL_DAY_TYPE:		.DB 0x02			;	size
								.DB	0x18			;   day of bill
								.DB	0x00			;	month of 0-Each,1-Odd,2-Even
;#6								
FLASH_ACTIVE_CAL_NAME:			.DB 0x36			;	size
								.DB	'A','C','A','L','1',0x00,0x00,0x00		;	Active Calander name
								.DB	'A','S','E','A','S','O','N',0x00		;	Active Season1 name
								.DB	'W','E','E','K','1',0x00,0x00,0x00		;	Active Season1 week name
								.DB	0x01,0x01,0x01,0x01,0x01,0x01,0x01		;	Active Season1 day#
								.DB	'B','S','E','A','S','O','N',0x00		;	Active Season2 name
								.DB	'W','E','E','K','2',0x00,0x00,0x00		;	Active Season2 week name
								.DB	0x02,0x02,0x02,0x02,0x02,0x02,0x02		;	Active Season2 day#
;#7
FLASH_ACTIVE_SEASON1_ZONE_TIME:	.DB 0x19			;	size
								.DB	0x00,0x00		;	TZ1-0
								.DB	0x68,0x01		;	TZ2-360
								.DB	0x1C,0x02		;	TZ3-540
								.DB	0x56,0x04		;	TZ4-1110
								.DB	0x74,0x04		;	TZ5-1140
								.DB	0x0A,0x05		;	TZ6-1290
								.DB	0x28,0x05		;	TZ7-1320
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x07			; 	Total zones-7
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x03			; 	TZ3-zone no
								.DB	0x04			; 	TZ4-zone no
								.DB	0x05			; 	TZ5-zone no
								.DB	0x06			; 	TZ6-zone no
								.DB	0x07			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#8								
FLASH_ACTIVE_SEASON2_ZONE_TIME:	.DB 0x19			;	size
								.DB	0x00,0x00		;	TZ1-0
								.DB	0x68,0x01		;	TZ2-360
								.DB	0x58,0x02		;	TZ3-600
								.DB	0xFC,0x03		;	TZ4-1020
								.DB	0xEC,0x04		;	TZ5-1260
								.DB	0x00,0x00		;	TZ6-0
								.DB	0x00,0x00		;	TZ7-0
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x05			; 	Total zones-5
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x03			; 	TZ3-zone no
								.DB	0x04			; 	TZ4-zone no
								.DB	0x01			; 	TZ5-zone no
								.DB	0x00			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#9								
FLASH_ACTIVE_SEASON_TIME:		.DB 0x07			;	size
								.DB	0x01			; 	season1-start day
								.DB	0x01			; 	season1-start month
								.DB	0x01			; 	season2-start day
								.DB	0x01			; 	season2-start month
								.DB	0x01			;	total seasons
								.DB	0x01			;	seasons seq-1
								.DB	0x01			;	seasons seq-2
;#10								
FLASH_PASSIVE_CAL_NAME:			.DB 0x36			;	size
								.DB	'B','C','A','L','1',0x00,0x00,0x00		;	Passive Calander name
								.DB	'A','S','E','A','S','O','N',0x00		;	Passive Season name
								.DB	'W','E','E','K','1',0x00,0x00,0x00		;	Passive week name
								.DB	0x01,0x01,0x01,0x01,0x01,0x01,0x01		;	Passive Season1 day#
								.DB	'B','S','E','A','S','O','N',0x00		;	Passive Season2 name
								.DB	'W','E','E','K','2',0x00,0x00,0x00		;	Passive Season2 week name
								.DB	0x01,0x01,0x01,0x01,0x01,0x01,0x01		;	Passive Season2 day#
;#11								
FLASH_PASSIVE_SEASON1_ZONE_TIME:	.DB 0x19			;	size
								.DB	0x68,0x01		;	TZ1-360
								.DB	0x38,0x04		;	TZ2-1080
								.DB	0x28,0x05		;	TZ3-1320
								.DB	0x00,0x00		;	TZ4-0
								.DB	0x00,0x00		;	TZ5-0
								.DB	0x00,0x00		;	TZ6-0
								.DB	0x00,0x00		;	TZ7-0
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x03			; 	Total zones-3
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x03			; 	TZ3-zone no
								.DB	0x00			; 	TZ4-zone no
								.DB	0x00			; 	TZ5-zone no
								.DB	0x00			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#12								
FLASH_PASSIVE_SEASON2_ZONE_TIME:	.DB 0x19			;	size
								.DB	0x00,0x00		;	TZ1-00
								.DB	0x38,0x04		;	TZ2-1080
								.DB	0x00,0x00		;	TZ3-0
								.DB	0x00,0x00		;	TZ4-0
								.DB	0x00,0x00		;	TZ5-0
								.DB	0x00,0x00		;	TZ6-0
								.DB	0x00,0x00		;	TZ7-0
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x02			; 	Total zones-2
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x00			; 	TZ3-zone no
								.DB	0x00			; 	TZ4-zone no
								.DB	0x00			; 	TZ5-zone no
								.DB	0x00			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#13								
FLASH_PASSIVE_SEASON_TIME:		.DB 0x07			;	size
								.DB	0x01			; 	season1-start day
								.DB	0x01			; 	season1-start month
								.DB	0x01			; 	season2-start day
								.DB	0x01			; 	season2-start month
								.DB	0x01			;	total seasons
								.DB	0x01			;	seasons seq-1
								.DB	0x01			;	seasons seq-2
;#14
FLASH_IS_MEMORY_CLEARED:		.DB	0x04			;size
								.DB	0x04
								.DB	0x03
								.DB	0x02
								.DB	0x01
;#15
FLASH_DEFAULT_LLS_KEY:			.DB	0x08			;size
								.DB	'A'
								.DB	'B'
								.DB	'C'
								.DB	'D'
								.DB	'E'
								.DB	'F'
								.DB	'G'
								.DB	'H'
;#16
FLASH_DEFAULT_HLS_KEY:			.DB	0x10			;size
								.DB	'R'
								.DB	'E'
								.DB	'N'
								.DB	'E'
								.DB	'S'
								.DB	'A'
								.DB	'S'
								.DB	'_'
								.DB	'P'
								.DB	'6'
								.DB	'w'
								.DB	'R'
								.DB	'J'
								.DB	'2'
								.DB	'1'
								.DB	'F'
;#17
FLASH_DEFAULT_KONO:				.DB	0x0A			;size
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								
							
$ENDIF
$IF (ASM_TARGET_BOARD_WRANGAL==1)
; parameters' catogery defined in following format:1- byte-no of bytes, values
; lower bytes first
;#1
FLASH_PRE_BILL_SINGLE_ACTION:	.DB	0x05,0x00,0x00,0x00,0x00,0xFF
;#2
FLASH_PASSIVE_SEA_ACT_TIME:		.DB	0x05,0x00,0x00,0x00,0x00,0xFF
;#3
FLASH_MD_PERIOD:				.DB	0x02,0x08,0x07	;	1800
;#4
FLASH_LS_PERIOD:				.DB 0x02,0x02,0x00	;	1800
;#5
FLASH_PRE_BILL_DAY_TYPE:		.DB 0x02			;	size
								.DB	0x01			;   day of bill
								.DB	0x00			;	month of 0-Each,1-Odd,2-Even
;#6								
FLASH_ACTIVE_CAL_NAME:			.DB 0x36			;	size
								.DB	'A','C','A','L','1',0x00,0x00,0x00		;	Active Calander name
								.DB	'A','S','E','A','S','O','N',0x00		;	Active Season1 name
								.DB	'W','E','E','K','1',0x00,0x00,0x00		;	Active Season1 week name
								.DB	0x01,0x01,0x01,0x01,0x01,0x01,0x01		;	Active Season1 day#
								.DB	'B','S','E','A','S','O','N',0x00		;	Active Season2 name
								.DB	'W','E','E','K','2',0x00,0x00,0x00		;	Active Season2 week name
								.DB	0x02,0x02,0x02,0x02,0x02,0x02,0x02		;	Active Season2 day#
;#7
FLASH_ACTIVE_SEASON1_ZONE_TIME:	.DB 0x19			;	size
								.DB	0x00,0x00		;	TZ1-0
								.DB	0x68,0x01		;	TZ2-360
								.DB	0x58,0x02		;	TZ3-540
								.DB	0x48,0x03		;	TZ4-1110
								.DB	0x38,0x04		;	TZ5-1140
								.DB	0x28,0x05		;	TZ6-1290
								.DB	0x00,0x00		;	TZ7-1320
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x06			; 	Total zones-6
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x03			; 	TZ3-zone no
								.DB	0x04			; 	TZ4-zone no
								.DB	0x05			; 	TZ5-zone no
								.DB	0x06			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#8								
FLASH_ACTIVE_SEASON2_ZONE_TIME:	.DB 0x19			;	size
								.DB	0x00,0x00		;	TZ1-0
								.DB	0x68,0x01		;	TZ2-360
								.DB	0x58,0x02		;	TZ3-600
								.DB	0xFC,0x03		;	TZ4-1020
								.DB	0xEC,0x04		;	TZ5-1260
								.DB	0x00,0x00		;	TZ6-0
								.DB	0x00,0x00		;	TZ7-0
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x05			; 	Total zones-5
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x03			; 	TZ3-zone no
								.DB	0x04			; 	TZ4-zone no
								.DB	0x01			; 	TZ5-zone no
								.DB	0x00			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#9								
FLASH_ACTIVE_SEASON_TIME:		.DB 0x07			;	size
								.DB	0x01			; 	season1-start day
								.DB	0x01			; 	season1-start month
								.DB	0x01			; 	season2-start day
								.DB	0x01			; 	season2-start month
								.DB	0x01			;	total seasons
								.DB	0x01			;	seasons seq-1
								.DB	0x01			;	seasons seq-2
;#10								
FLASH_PASSIVE_CAL_NAME:			.DB 0x36			;	size
								.DB	'B','C','A','L','1',0x00,0x00,0x00		;	Passive Calander name
								.DB	'A','S','E','A','S','O','N',0x00		;	Passive Season name
								.DB	'W','E','E','K','1',0x00,0x00,0x00		;	Passive week name
								.DB	0x01,0x01,0x01,0x01,0x01,0x01,0x01		;	Passive Season1 day#
								.DB	'B','S','E','A','S','O','N',0x00		;	Passive Season2 name
								.DB	'W','E','E','K','2',0x00,0x00,0x00		;	Passive Season2 week name
								.DB	0x01,0x01,0x01,0x01,0x01,0x01,0x01		;	Passive Season2 day#
;#11								
FLASH_PASSIVE_SEASON1_ZONE_TIME:	.DB 0x19			;	size
								.DB	0x00,0x00		;	TZ1-0
								.DB	0x68,0x01		;	TZ2-360
								.DB	0x58,0x02		;	TZ3-540
								.DB	0x48,0x03		;	TZ4-1110
								.DB	0x38,0x04		;	TZ5-1140
								.DB	0x28,0x05		;	TZ6-1290
								.DB	0x00,0x00		;	TZ7-1320
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x06			; 	Total zones-6
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x03			; 	TZ3-zone no
								.DB	0x04			; 	TZ4-zone no
								.DB	0x05			; 	TZ5-zone no
								.DB	0x06			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#12								
FLASH_PASSIVE_SEASON2_ZONE_TIME:	.DB 0x19			;	size
								.DB	0x00,0x00		;	TZ1-00
								.DB	0x38,0x04		;	TZ2-1080
								.DB	0x00,0x00		;	TZ3-0
								.DB	0x00,0x00		;	TZ4-0
								.DB	0x00,0x00		;	TZ5-0
								.DB	0x00,0x00		;	TZ6-0
								.DB	0x00,0x00		;	TZ7-0
								.DB	0x00,0x00		;	TZ8-0
								.DB	0x02			; 	Total zones-2
								.DB	0x01			; 	TZ1-zone no
								.DB	0x02			; 	TZ2-zone no
								.DB	0x00			; 	TZ3-zone no
								.DB	0x00			; 	TZ4-zone no
								.DB	0x00			; 	TZ5-zone no
								.DB	0x00			; 	TZ6-zone no
								.DB	0x00			; 	TZ7-zone no
								.DB	0x00			; 	TZ8-zone no
;#13								
FLASH_PASSIVE_SEASON_TIME:		.DB 0x07			;	size
								.DB	0x01			; 	season1-start day
								.DB	0x01			; 	season1-start month
								.DB	0x01			; 	season2-start day
								.DB	0x01			; 	season2-start month
								.DB	0x01			;	total seasons
								.DB	0x01			;	seasons seq-1
								.DB	0x01			;	seasons seq-2
;#14
FLASH_IS_MEMORY_CLEARED:		.DB	0x04			;size
								.DB	0x04
								.DB	0x03
								.DB	0x02
								.DB	0x01
;#15
FLASH_DEFAULT_LLS_KEY:			.DB	0x08			;size
								.DB	'A'
								.DB	'B'
								.DB	'C'
								.DB	'D'
								.DB	'E'
								.DB	'F'
								.DB	'G'
								.DB	'H'
;#16
FLASH_DEFAULT_HLS_KEY:			.DB	0x10			;size
								.DB	'R'
								.DB	'E'
								.DB	'N'
								.DB	'E'
								.DB	'S'
								.DB	'A'
								.DB	'S'
								.DB	'_'
								.DB	'P'
								.DB	'6'
								.DB	'w'
								.DB	'R'
								.DB	'J'
								.DB	'2'
								.DB	'1'
								.DB	'F'
;#17
FLASH_DEFAULT_KONO:				.DB	0x0A			;size
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
								.DB '1'
;#18
FLASH_DEFAULT_LAG_ONLY_CONFIG:						.DB	0x01,0x00	;	0								
							
$ENDIF															
