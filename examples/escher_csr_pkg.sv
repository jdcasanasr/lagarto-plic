package escher_csr_pkg;

// riscv_pkg.sv :
parameter XLEN = 64; 
parameter VLEN = 128;
parameter MLEN = VLEN/8;
parameter OPCODE_WIDTH = 6;
parameter REG_WIDTH = 5;
parameter NUM_ISA_REGISTERS = 32;
parameter NUM_ISA_VREGISTERS = 32;
parameter INST_SIZE = 32;

// Status flags
typedef struct packed {
    logic NV; // Invalid
    logic DZ; // Divide by zero
    logic OF; // Overflow
    logic UF; // Underflow
    logic NX; // Inexact
} fp_status_t;

// By RISCV ISA, exceptions are 64 bits
typedef enum logic[XLEN-1:0] {
    INSTR_ADDR_MISALIGNED   = 64'h00,
    INSTR_ACCESS_FAULT      = 64'h01,
    ILLEGAL_INSTR           = 64'h02,
    BREAKPOINT              = 64'h03,
    LD_ADDR_MISALIGNED      = 64'h04,
    LD_ACCESS_FAULT         = 64'h05,
    ST_AMO_ADDR_MISALIGNED  = 64'h06,
    ST_AMO_ACCESS_FAULT     = 64'h07,
    USER_ECALL              = 64'h08,
    SUPERVISOR_ECALL        = 64'h09,
    INSTR_PAGE_FAULT        = 64'h0C,
    LD_PAGE_FAULT           = 64'h0D,
    ST_AMO_PAGE_FAULT       = 64'h0F,
    DEBUG_REQUEST           = 64'h18,
    NONE                    = 64'hFF
} exception_cause_t;

// --------------------
// Privilege Spec
// --------------------
typedef enum logic[1:0] {
    PRIV_LVL_M = 2'b11,
    PRIV_LVL_S = 2'b01,
    PRIV_LVL_U = 2'b00
} priv_lvl_t;

// type which holds xlen
typedef enum logic [1:0] {
    XLEN_32  = 2'b01,
    XLEN_64  = 2'b10,
    XLEN_128 = 2'b11
} xlen_t;

typedef enum logic [1:0] {
    Off_State     = 2'b00,
    Initial_State = 2'b01,
    Clean_State   = 2'b10,
    Dirty_State   = 2'b11
} xs_t;

typedef struct packed {
    logic         sd;     // signal dirty state - read-only
    logic [62:36] wpri4;  // writes preserved reads ignored
    xlen_t        sxl;    // variable supervisor mode xlen - hardwired to zero
    xlen_t        uxl;    // variable user mode xlen - hardwired to zero
    logic [8:0]   wpri3;  // writes preserved reads ignored
    logic         tsr;    // trap sret
    logic         tw;     // time wait
    logic         tvm;    // trap virtual memory
    logic         mxr;    // make executable readable
    logic         sum;    // permit supervisor user memory access
    logic         mprv;   // modify privilege - privilege level for ld/st
    xs_t          xs;     // extension register - hardwired to zero
    xs_t          fs;     // floating point extension register
    priv_lvl_t    mpp;    // holds the previous privilege mode up to machine
    logic [1:0]   vs;  // writes preserved reads ignored
    logic         spp;    // holds the previous privilege mode up to supervisor
    logic         mpie;   // machine interrupts enable bit active prior to trap
    logic         wpri1;  // writes preserved reads ignored
    logic         spie;   // supervisor interrupts enable bit active prior to trap
    logic         upie;   // user interrupts enable bit active prior to trap - hardwired to zero
    logic         mie;    // machine interrupts enable
    logic         wpri0;  // writes preserved reads ignored
    logic         sie;    // supervisor interrupts enable
    logic         uie;    // user interrupts enable - hardwired to zero
} status_rv64_t;

typedef struct packed {
    logic         sd;     // signal dirty - read-only - hardwired zero
    logic [7:0]   wpri3;  // writes preserved reads ignored
    logic         tsr;    // trap sret
    logic         tw;     // time wait
    logic         tvm;    // trap virtual memory
    logic         mxr;    // make executable readable
    logic         sum;    // permit supervisor user memory access
    logic         mprv;   // modify privilege - privilege level for ld/st
    logic [1:0]   xs;     // extension register - hardwired to zero
    logic [1:0]   fs;     // extension register - hardwired to zero
    priv_lvl_t    mpp;    // holds the previous privilege mode up to machine
    logic [1:0]   wpri2;  // writes preserved reads ignored
    logic         spp;    // holds the previous privilege mode up to supervisor
    logic         mpie;   // machine interrupts enable bit active prior to trap
    logic         wpri1;  // writes preserved reads ignored
    logic         spie;   // supervisor interrupts enable bit active prior to trap
    logic         upie;   // user interrupts enable bit active prior to trap - hardwired to zero
    logic         mie;    // machine interrupts enable
    logic         wpri0;  // writes preserved reads ignored
    logic         sie;    // supervisor interrupts enable
    logic         uie;    // user interrupts enable - hardwired to zero
} status_rv32_t;

// TODO: Plan a way to consolidate this register with riscv_pkg, can be in both places, but names and values must match, rv64_satp ang rv32_satp seems a good approach.
typedef struct packed {
    logic [3:0]  mode;
    logic [15:0] asid;
    logic [43:0] ppn;
} satp_t;

localparam int unsigned IRQ_S_SOFT  = 1;
localparam int unsigned IRQ_M_SOFT  = 3;
localparam int unsigned IRQ_S_TIMER = 5;
localparam int unsigned IRQ_M_TIMER = 7;
localparam int unsigned IRQ_S_EXT   = 9;
localparam int unsigned IRQ_M_EXT   = 11;
localparam int unsigned IRQ_LCOF    = 13;

localparam logic [63:0] MIP_SSIP = 1 << IRQ_S_SOFT;
localparam logic [63:0] MIP_MSIP = 1 << IRQ_M_SOFT;
localparam logic [63:0] MIP_STIP = 1 << IRQ_S_TIMER;
localparam logic [63:0] MIP_MTIP = 1 << IRQ_M_TIMER;
localparam logic [63:0] MIP_SEIP = 1 << IRQ_S_EXT;
localparam logic [63:0] MIP_MEIP = 1 << IRQ_M_EXT;
localparam logic [63:0] MIP_LCOFIP = 1 << IRQ_LCOF;

localparam logic [63:0] S_SW_INTERRUPT    = (1 << 63) | IRQ_S_SOFT;
localparam logic [63:0] M_SW_INTERRUPT    = (1 << 63) | IRQ_M_SOFT;
localparam logic [63:0] S_TIMER_INTERRUPT = (1 << 63) | IRQ_S_TIMER;
localparam logic [63:0] M_TIMER_INTERRUPT = (1 << 63) | IRQ_M_TIMER;
localparam logic [63:0] S_EXT_INTERRUPT   = (1 << 63) | IRQ_S_EXT;
localparam logic [63:0] M_EXT_INTERRUPT   = (1 << 63) | IRQ_M_EXT;
localparam logic [63:0] LCOF_INTERRUPT    = (1 << 63) | IRQ_LCOF;

// -----
// CSRs
// -----
typedef enum logic [11:0] {
    // Floating-Point CSRs
    CSR_FFLAGS         = 12'h001,
    CSR_FRM            = 12'h002,
    CSR_FCSR           = 12'h003,
    // Custom read/write (User Mode)
    CSR_FTRAN          = 12'h800,   // TODO: custom CSR not used anywhere in the library, can be removed
    CSR_UARCH_CFG      = 12'h801,   // uARCH config CSR
    CSR_ILA_DBG_CFG    = 12'h802,   // Control register to configure ILA trigger handler.
    CSR_ILA_CND_PC     = 12'h803,   // ILA Conditional trigger program counter value.
    CSR_ILA_CND_MADDR  = 12'h804,   // ILA Conditional trigger memory address value.
    CSR_ILA_CND_TIME   = 12'h805,   // ILA Conditional trigger TIME register value.
    CSR_ILA_CND_MASK   = 12'h806,   // ILA Conditional register mask value.
    // Supervisor Mode CSRs
    CSR_SSTATUS        = 12'h100,
    CSR_SIE            = 12'h104,
    CSR_STVEC          = 12'h105,
    CSR_SCOUNTEREN     = 12'h106,
    CSR_SSCRATCH       = 12'h140,
    CSR_SEPC           = 12'h141,
    CSR_SCAUSE         = 12'h142,
    CSR_STVAL          = 12'h143,
    CSR_SIP            = 12'h144,
    CSR_SATP           = 12'h180,
    CSR_SCOUNTOVF      = 12'hDA0,
    // Machine Mode CSRs
    CSR_MSTATUS        = 12'h300,
    CSR_MISA           = 12'h301,
    CSR_MEDELEG        = 12'h302,
    CSR_MIDELEG        = 12'h303,
    CSR_MIE            = 12'h304,
    CSR_MTVEC          = 12'h305,
    CSR_MCOUNTEREN     = 12'h306,
    CSR_MCOUNTINHIBIT  = 12'h320,  // Not in the ISA
    CSR_MHPM_EVENT_3   = 12'h323,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_4   = 12'h324,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_5   = 12'h325,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_6   = 12'h326,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_7   = 12'h327,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_8   = 12'h328,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_9   = 12'h329,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_10  = 12'h32a,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_11  = 12'h32b,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_12  = 12'h32c,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_13  = 12'h32d,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_14  = 12'h32e,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_15  = 12'h32f,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_16  = 12'h330,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_17  = 12'h331,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_18  = 12'h332,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_19  = 12'h333,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_20  = 12'h334,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_21  = 12'h335,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_22  = 12'h336,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_23  = 12'h337,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_24  = 12'h338,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_25  = 12'h339,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_26  = 12'h33a,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_27  = 12'h33b,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_28  = 12'h33c,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_29  = 12'h33d,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_30  = 12'h33e,  //Machine performance monitoring Event Selector
    CSR_MHPM_EVENT_31  = 12'h33f,  //Machine performance monitoring Event Selector
    CSR_MSCRATCH       = 12'h340,
    CSR_MEPC           = 12'h341,
    CSR_MCAUSE         = 12'h342,
    CSR_MTVAL          = 12'h343,
    CSR_MIP            = 12'h344,
    CSR_MVENDORID      = 12'hF11,
    CSR_MARCHID        = 12'hF12,
    CSR_MIMPID         = 12'hF13,
    CSR_MHARTID        = 12'hF14,
    `ifdef PITON_CINCORANCH
    CSR_MBOOT_MAIN_ID  = 12'hFC0,
    `endif
    CSR_MCYCLE         = 12'hB00,
    CSR_MINSTRET       = 12'hB02,
    // Performance counters (Machine Mode)
    CSR_MHPM_COUNTER_3  = 12'hB03,
    CSR_MHPM_COUNTER_4  = 12'hB04,
    CSR_MHPM_COUNTER_5  = 12'hB05,
    CSR_MHPM_COUNTER_6  = 12'hB06,
    CSR_MHPM_COUNTER_7  = 12'hB07,
    CSR_MHPM_COUNTER_8  = 12'hB08,
    CSR_MHPM_COUNTER_9  = 12'hB09,
    CSR_MHPM_COUNTER_10 = 12'hB0A,
    CSR_MHPM_COUNTER_11 = 12'hB0B,
    CSR_MHPM_COUNTER_12 = 12'hB0C,
    CSR_MHPM_COUNTER_13 = 12'hB0D,
    CSR_MHPM_COUNTER_14 = 12'hB0E,
    CSR_MHPM_COUNTER_15 = 12'hB0F,
    CSR_MHPM_COUNTER_16 = 12'hB10,
    CSR_MHPM_COUNTER_17 = 12'hB11,
    CSR_MHPM_COUNTER_18 = 12'hB12,
    CSR_MHPM_COUNTER_19 = 12'hB13,
    CSR_MHPM_COUNTER_20 = 12'hB14,
    CSR_MHPM_COUNTER_21 = 12'hB15,
    CSR_MHPM_COUNTER_22 = 12'hB16,
    CSR_MHPM_COUNTER_23 = 12'hB17,
    CSR_MHPM_COUNTER_24 = 12'hB18,
    CSR_MHPM_COUNTER_25 = 12'hB19,
    CSR_MHPM_COUNTER_26 = 12'hB1A,
    CSR_MHPM_COUNTER_27 = 12'hB1B,
    CSR_MHPM_COUNTER_28 = 12'hB1C,
    CSR_MHPM_COUNTER_29 = 12'hB1D,
    CSR_MHPM_COUNTER_30 = 12'hB1E,
    CSR_MHPM_COUNTER_31 = 12'hB1F,
    // Cache Control (platform specifc)
    CSR_DCACHE         = 12'h701,
    CSR_ICACHE         = 12'h700,
    // Triggers
    CSR_TSELECT        = 12'h7A0,
    CSR_TDATA1         = 12'h7A1,
    CSR_TDATA2         = 12'h7A2,
    CSR_TDATA3         = 12'h7A3,
    CSR_TINFO          = 12'h7A4,
    // Debug CSR
    CSR_DCSR           = 12'h7b0,
    CSR_DPC            = 12'h7b1,
    CSR_DSCRATCH0      = 12'h7b2, // optional
    CSR_DSCRATCH1      = 12'h7b3, // optional
    // Counters and Timers (User Mode - R/O Shadows)
    CSR_CYCLE          = 12'hC00,
    CSR_TIME           = 12'hC01,
    CSR_INSTRET        = 12'hC02,
    // Performance counters (User Mode - R/O Shadows)
    CSR_HPM_COUNTER_3  = 12'hC03,
    CSR_HPM_COUNTER_4  = 12'hC04,
    CSR_HPM_COUNTER_5  = 12'hC05,
    CSR_HPM_COUNTER_6  = 12'hC06,
    CSR_HPM_COUNTER_7  = 12'hC07,
    CSR_HPM_COUNTER_8  = 12'hC08,
    CSR_HPM_COUNTER_9  = 12'hC09,
    CSR_HPM_COUNTER_10 = 12'hC0A,
    CSR_HPM_COUNTER_11 = 12'hC0B,
    CSR_HPM_COUNTER_12 = 12'hC0C,
    CSR_HPM_COUNTER_13 = 12'hC0D,
    CSR_HPM_COUNTER_14 = 12'hC0E,
    CSR_HPM_COUNTER_15 = 12'hC0F,
    CSR_HPM_COUNTER_16 = 12'hC10,
    CSR_HPM_COUNTER_17 = 12'hC11,
    CSR_HPM_COUNTER_18 = 12'hC12,
    CSR_HPM_COUNTER_19 = 12'hC13,
    CSR_HPM_COUNTER_20 = 12'hC14,
    CSR_HPM_COUNTER_21 = 12'hC15,
    CSR_HPM_COUNTER_22 = 12'hC16,
    CSR_HPM_COUNTER_23 = 12'hC17,
    CSR_HPM_COUNTER_24 = 12'hC18,
    CSR_HPM_COUNTER_25 = 12'hC19,
    CSR_HPM_COUNTER_26 = 12'hC1A,
    CSR_HPM_COUNTER_27 = 12'hC1B,
    CSR_HPM_COUNTER_28 = 12'hC1C,
    CSR_HPM_COUNTER_29 = 12'hC1D,
    CSR_HPM_COUNTER_30 = 12'hC1E,
    CSR_HPM_COUNTER_31 = 12'hC1F,

    CSR_MEM_MAP_0   = 12'h7C0,  // MEM space
    CSR_MEM_MAP_1   = 12'h7C1,  // MEM space
    CSR_MEM_MAP_2   = 12'h7C2,  // MEM space
    CSR_MEM_MAP_3   = 12'h7C3,  // MEM space
    CSR_MEM_MAP_4   = 12'h7C4,  // MEM space
    CSR_MEM_MAP_5   = 12'h7C5,  // MEM space
    CSR_MEM_MAP_6   = 12'h7C6,  // MEM space
    CSR_MEM_MAP_7   = 12'h7C7,  // MEM space
    CSR_MEM_MAP_8   = 12'h7C8,  // MEM space
    CSR_MEM_MAP_9   = 12'h7C9,  // MEM space
    CSR_MEM_MAP_10  = 12'h7CA,  // MEM space
    CSR_MEM_MAP_11  = 12'h7CB,  // MEM space
    CSR_MEM_MAP_12  = 12'h7CC,  // MEM space
    CSR_MEM_MAP_13  = 12'h7CD,  // MEM space
    CSR_MEM_MAP_14  = 12'h7CE,  // MEM space
    CSR_MEM_MAP_15  = 12'h7CF,  // MEM space

    CSR_IO_MAP_0    = 12'h7D0,  // IO space
    CSR_IO_MAP_1    = 12'h7D1,  // IO space
    CSR_IO_MAP_2    = 12'h7D2,  // IO space
    CSR_IO_MAP_3    = 12'h7D3,  // IO space
    CSR_IO_MAP_4    = 12'h7D4,  // IO space
    CSR_IO_MAP_5    = 12'h7D5,  // IO space
    CSR_IO_MAP_6    = 12'h7D6,  // IO space
    CSR_IO_MAP_7    = 12'h7D7,  // IO space
    CSR_IO_MAP_8    = 12'h7D8,  // IO space
    CSR_IO_MAP_9    = 12'h7D9,  // IO space
    CSR_IO_MAP_10   = 12'h7DA,  // IO space
    CSR_IO_MAP_11   = 12'h7DB,  // IO space
    CSR_IO_MAP_12   = 12'h7DC,  // IO space
    CSR_IO_MAP_13   = 12'h7DD,  // IO space
    CSR_IO_MAP_14   = 12'h7DE,  // IO space
    CSR_IO_MAP_15   = 12'h7DF,  // IO space

    CSR_IRQ_MAP_0   = 12'h7E0,  // IRQ space
    CSR_IRQ_MAP_1   = 12'h7E1,  // IRQ space
    CSR_IRQ_MAP_2   = 12'h7E2,  // IRQ space
    CSR_IRQ_MAP_3   = 12'h7E3,  // IRQ space
    CSR_IRQ_MAP_4   = 12'h7E4,  // IRQ space
    CSR_IRQ_MAP_5   = 12'h7E5,  // IRQ space
    CSR_IRQ_MAP_6   = 12'h7E6,  // IRQ space
    CSR_IRQ_MAP_7   = 12'h7E7,  // IRQ space
    CSR_IRQ_MAP_8   = 12'h7E8,  // IRQ space
    CSR_IRQ_MAP_9   = 12'h7E9,  // IRQ space
    CSR_IRQ_MAP_10  = 12'h7EA,  // IRQ space
    CSR_IRQ_MAP_11  = 12'h7EB,  // IRQ space
    CSR_IRQ_MAP_12  = 12'h7EC,  // IRQ space
    CSR_IRQ_MAP_13  = 12'h7ED,  // IRQ space
    CSR_IRQ_MAP_14  = 12'h7EE,  // IRQ space
    CSR_IRQ_MAP_15  = 12'h7EF,  // IRQ space

    CSR_PMPCFG_0    = 12'h3A0,  //Physical memory protection config
    CSR_PMPCFG_1    = 12'h3A1,  //Physical memory protection config
    CSR_PMPCFG_2    = 12'h3A2,  //Physical memory protection config
    CSR_PMPCFG_3    = 12'h3A3,  //Physical memory protection config

    CSR_PMPADDR_0   = 12'h3B0,  //Physical memory protection addr
    CSR_PMPADDR_1   = 12'h3B1,  //Physical memory protection addr
    CSR_PMPADDR_2   = 12'h3B2,  //Physical memory protection addr
    CSR_PMPADDR_3   = 12'h3B3,  //Physical memory protection addr
    CSR_PMPADDR_4   = 12'h3B4,  //Physical memory protection addr
    CSR_PMPADDR_5   = 12'h3B5,  //Physical memory protection addr
    CSR_PMPADDR_6   = 12'h3B6,  //Physical memory protection addr
    CSR_PMPADDR_7   = 12'h3B7,  //Physical memory protection addr
    CSR_PMPADDR_8   = 12'h3B8,  //Physical memory protection addr
    CSR_PMPADDR_9   = 12'h3B9,  //Physical memory protection addr
    CSR_PMPADDR_10  = 12'h3BA,  //Physical memory protection addr
    CSR_PMPADDR_11  = 12'h3BB,  //Physical memory protection addr
    CSR_PMPADDR_12  = 12'h3BC,  //Physical memory protection addr
    CSR_PMPADDR_13  = 12'h3BD,  //Physical memory protection addr
    CSR_PMPADDR_14  = 12'h3BE,  //Physical memory protection addr
    CSR_PMPADDR_15  = 12'h3BF,  //Physical memory protection addr

    TO_HOST         = 12'h9F0,  // to host csr used for simulation
    FROM_HOST       = 12'h9F1,  // from host csr used for simulation

    CSR_VL          = 12'hC20,  // Vector extension CSR
    CSR_VTYPE       = 12'hC21,  // Vector extension CSR

    CSR_HYPERRAM_CONFIG = 12'h7F0,  // HyperRAM Configuration CSR
    CSR_CNM_CONFIG = 12'h7F1, 	// CNM Peripherals Configuration CSR 
    CSR_SPI_CONFIG = 12'h7F2  // SPI Configuration CSR
} csr_reg_t;

localparam logic [63:0] SSTATUS_UIE    = 64'h00000001;
localparam logic [63:0] SSTATUS_SIE    = 64'h00000002;
localparam logic [63:0] SSTATUS_SPIE   = 64'h00000020;
localparam logic [63:0] SSTATUS_SPP    = 64'h00000100;
localparam logic [63:0] SSTATUS_FS     = 64'h00006000;
localparam logic [63:0] SSTATUS_XS     = 64'h00018000;
localparam logic [63:0] SSTATUS_SUM    = 64'h00040000;
localparam logic [63:0] SSTATUS_MXR    = 64'h00080000;
localparam logic [63:0] SSTATUS_UPIE   = 64'h00000010;
localparam logic [63:0] SSTATUS_UXL    = 64'h0000000300000000;
localparam logic [63:0] SSTATUS64_SD   = 64'h8000000000000000;
localparam logic [63:0] SSTATUS32_SD   = 64'h80000000;
localparam logic [63:0] SSTATUS64_WPRI = 64'h7ffffffdfff398cc;

localparam logic [63:0] MSTATUS_UIE    = 64'h00000001;
localparam logic [63:0] MSTATUS_SIE    = 64'h00000002;
localparam logic [63:0] MSTATUS_HIE    = 64'h00000004;
localparam logic [63:0] MSTATUS_MIE    = 64'h00000008;
localparam logic [63:0] MSTATUS_UPIE   = 64'h00000010;
localparam logic [63:0] MSTATUS_SPIE   = 64'h00000020;
localparam logic [63:0] MSTATUS_HPIE   = 64'h00000040;
localparam logic [63:0] MSTATUS_MPIE   = 64'h00000080;
localparam logic [63:0] MSTATUS_SPP    = 64'h00000100;
localparam logic [63:0] MSTATUS_HPP    = 64'h00000600;
localparam logic [63:0] MSTATUS_MPP    = 64'h00001800;
localparam logic [63:0] MSTATUS_FS     = 64'h00006000;
localparam logic [63:0] MSTATUS_XS     = 64'h00018000;
localparam logic [63:0] MSTATUS_MPRV   = 64'h00020000;
localparam logic [63:0] MSTATUS_SUM    = 64'h00040000;
localparam logic [63:0] MSTATUS_MXR    = 64'h00080000;
localparam logic [63:0] MSTATUS_TVM    = 64'h00100000;
localparam logic [63:0] MSTATUS_TW     = 64'h00200000;
localparam logic [63:0] MSTATUS_TSR    = 64'h00400000;
localparam logic [63:0] MSTATUS32_SD   = 64'h80000000;
localparam logic [64:0] MSTATUS32_WPRI = 64'h7f800044;
localparam logic [63:0] MSTATUS_UXL    = 64'h0000000300000000;
localparam logic [63:0] MSTATUS_SXL    = 64'h0000000C00000000;
localparam logic [63:0] MSTATUS64_SD   = 64'h8000000000000000;
localparam logic [63:0] MSTATUS64_WPRI = 64'h7ffffff5ff800044;

// decoded CSR address
typedef struct packed {
    logic [1:0]  rw;
    priv_lvl_t   priv_lvl;
    logic  [7:0] address;
} csr_addr_t;

typedef union packed {
    csr_reg_t   address;
    csr_addr_t  csr_decode;
} csr_t;

// Floating-Point control and status register (32-bit!)
typedef struct packed {
    logic [31:15] reserved;  // reserved for L extension, return 0 otherwise
    logic [6:0]   fprec;     // div/sqrt precision control
    logic [2:0]   frm;       // float rounding mode
    logic [4:0]   fflags;    // float exception flags
} fcsr_t;

// -----
// Custom uArch configuration register
// -----
typedef struct packed {
    logic [63:3]  reserved;
    logic [2:0]   bke_queues_inorder;     // Defines if bke queues issue inorder
} uarch_cfg_csr_t;

// -----
// Custom ILA debug configuration register
// -----
typedef struct packed {
    logic [63:15]   reserved;  
    logic [14:14]   ebrk;       // Exception breakpoint at PC CSR_ILA_CND_PC.     
    logic [13:13]   rdc;        // Trigger reduction configuration; 1: AND reduction, 0: OR reduction.
    logic [12:12]   tgb;        // Trigger recording behaviour; 1: Start on trigger 0: Stop on trigger.
    logic [11:9]    ctime;      // ILA recording conditional trigger by time register value.
    logic [8:6]     cmadr;      // ILA recording conditional trigger by memory address value.
    logic [5:3]     cpc;        // ILA recording conditional trigger by pc value.
    logic [2:2]     swtg;       // ILA recording software trigger.
    logic [1:1]     swen;       // ILA recording trigger enable by software.
    logic [0:0]     hwen;       // ILA recording trigger enable by hardware assertions.
} ila_dbg_cfg_csr_t;

// -----
// Debug
// -----
typedef struct packed {
    logic [31:28]     xdebugver;
    logic [27:16]     zero2;
    logic             ebreakm;
    logic             zero1;
    logic             ebreaks;
    logic             ebreaku;
    logic             stepie;
    logic             stopcount;
    logic             stoptime;
    logic [8:6]       cause;
    logic             zero0;
    logic             mprven;
    logic             nmip;
    logic             step;
    priv_lvl_t        prv;
} dcsr_t;

// def_pkg.sv :

localparam bit RVC = 1'b1; // Is C extension enabled

// Floating-point extensions configuration
localparam bit RVF = 1'b1; // Is F extension enabled
localparam bit RVD = 1'b1; // Is D extension enabled
localparam bit RVA = 1'b1; // Is A extension enabled
localparam bit RVV = 1'b0; // Is V extension enabled

// Transprecision floating-point extensions configuration
localparam bit XF16    = 1'b0; // Is half-precision float extension (Xf16) enabled
localparam bit XF16ALT = 1'b0; // Is alternative half-precision float extension (Xf16alt) enabled
localparam bit XF8     = 1'b0; // Is quarter-precision float extension (Xf8) enabled
localparam bit XFVEC   = 1'b0; // Is vectorial float extension (Xfvec) enabled

// Transprecision float unit
localparam int unsigned LAT_COMP_FP32    = 'd2;
localparam int unsigned LAT_COMP_FP64    = 'd3;
localparam int unsigned LAT_COMP_FP16    = 'd1;
localparam int unsigned LAT_COMP_FP16ALT = 'd1;
localparam int unsigned LAT_COMP_FP8     = 'd1;
localparam int unsigned LAT_DIVSQRT      = 'd2;
localparam int unsigned LAT_NONCOMP      = 'd1;
localparam int unsigned LAT_CONV         = 'd2;

// --------------------------------------
// vvvv Don't change these by hand! vvvv
localparam bit FP_PRESENT = RVF | RVD | XF16 | XF16ALT | XF8;

localparam bit V_PRESENT = RVV;

// Length of widest floating-point format
localparam FLEN    = RVD     ? 64 : // D ext.
                        RVF     ? 32 : // F ext.
                        XF16    ? 16 : // Xf16 ext.
                        XF16ALT ? 16 : // Xf16alt ext.
                        XF8     ? 8 :  // Xf8 ext.
                        0;             // Unused in case of no FP

localparam bit NSX = XF16 | XF16ALT | XF8 | XFVEC; // Are non-standard extensions present?

localparam bit RVFVEC     = RVF     & XFVEC & (FLEN>32); // FP32 vectors available if vectors and larger fmt enabled
localparam bit XF16VEC    = XF16    & XFVEC & (FLEN>16); // FP16 vectors available if vectors and larger fmt enabled
localparam bit XF16ALTVEC = XF16ALT & XFVEC & (FLEN>16); // FP16ALT vectors available if vectors and larger fmt enabled
localparam bit XF8VEC     = XF8     & XFVEC & (FLEN>8);  // FP8 vectors available if vectors and larger fmt enabled

localparam logic [63:0] ISA_CODE = (RVA <<  0)  // A - Atomic Instructions extension
                                    | (RVC <<  2)  // C - Compressed extension
                                    | (RVD <<  3)  // D - Double precsision floating-point extension
                                    | (RVF <<  5)  // F - Single precsision floating-point extension
                                    | (1   <<  8)  // I - RV32I/64I/128I base ISA
                                    | (1   << 12)  // M - Integer Multiply/Divide extension
                                    | (0   << 13)  // N - User level interrupts supported
                                    | (1   << 18)  // S - Supervisor mode implemented
                                    | (1   << 20)  // U - User mode implemented
                                    | (NSX << 23)  // X - Non-standard extensions present
                                    | (1   << 63); // RV64


// enables a commit log which matches spikes commit log format for easier trace comparison
localparam bit ENABLE_SPIKE_COMMIT_LOG = 1'b1;

// read mask for SSTATUS over MMSTATUS
localparam logic [63:0] SMODE_STATUS_READ_MASK = SSTATUS_UIE
                                                | SSTATUS_SIE
                                                | SSTATUS_SPIE
                                                | SSTATUS_SPP
                                                | SSTATUS_FS
                                                | SSTATUS_XS
                                                | SSTATUS_SUM
                                                | SSTATUS_MXR
                                                | SSTATUS_UPIE
                                                | SSTATUS_SPIE
                                                | SSTATUS_UXL
                                                | SSTATUS64_SD;

localparam logic [63:0] SMODE_STATUS_WRITE_MASK = SSTATUS_SIE
                                                | SSTATUS_SPIE
                                                | SSTATUS_SPP
                                                | SSTATUS_FS
                                                | SSTATUS_SUM
                                                | SSTATUS_MXR;
// exception
typedef struct packed {
        logic [63:0] cause; // cause of exception
        logic [63:0] tval;  // additional information of causing exception (e.g.: instruction causing it),
                            // address of LD/ST fault
        logic        valid;
} exception_t;

localparam logic [3:0] MODE_SV39 = 4'h8;
localparam logic [3:0] MODE_OFF = 4'h0;

// Bits required for representation of physical address space as 4K pages
// (e.g. 27*4K == 39bit address space).
localparam PPN4K_WIDTH = 38;

endpackage
