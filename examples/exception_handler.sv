import riscv_privileged::*;

module exception_handler
(
    input logic clock_i,
    input logic reset_ni,

    // Other ports.
);

    privilege_level_t privilege_level_r;

    // M-Mode Registers.
    // Machine Trap Setup.
    rv64_xstatus_t      mstatus_r   , mstatus_w   ;
    logic [MLEN - 1:0]  misa_r      , misa_w      ;
    logic [MLEN - 1:0]  mtvec_r     , mtvec_w     ;
    logic [MLEN - 1:0]  medeleg_r   , medeleg_w   ;
    logic [MLEN - 1:0]  mideleg_r   , mideleg_w   ;
    logic [MLEN - 1:0]  mie_r       , mie_w       ;
    logic [MLEN - 1:0]  mtvec_r     , mtvec_w     ;
    logic [MLEN - 1:0]  mcounteren_r, mcounteren_w;

    // Machine Trap Handling.
    logic [MLEN - 1:0]  mscratch_r, mscratch_w  ;
    logic [MLEN - 1:0]  mepc_r    , mepc_w      ;
    logic [MLEN - 1:0]  mcause_r  , mcause_w    ;
    logic [MLEN - 1:0]  mtval_r   , mtval_w     ;
    logic [MLEN - 1:0]  mip_r     , mip_w       ;
    logic [MLEN - 1:0]  mtinst_r  , mtinst_w    ;
    logic [MLEN - 1:0]  mtval2_r  , mtval2_w    ;
    
    // S-Mode Registers.
    // Supervisor Trap Setup.
    rv64_xstatus_t      sstatus_r   , sstatus_w   ;
    logic [MLEN - 1:0]  sie_r       , sie_w       ;
    logic [MLEN - 1:0]  stvec_r     , stvec_w     ;
    logic [MLEN - 1:0]  scounteren_r, scounteren_w;

    // Supervisor Trap Handling.
    logic [MLEN - 1:0]  sscratch_r , sscratch_w ;
    logic [MLEN - 1:0]  sepc_r     , sepc_w     ;
    logic [MLEN - 1:0]  scause_r   , scause_w   ;
    logic [MLEN - 1:0]  stval_r    , stval_w    ;
    logic [MLEN - 1:0]  sip_r      , sip_w      ;
    logic [MLEN - 1:0]  scountovf_r, scountovf_w;

    // CSR R/W Control Signals.
    logic [:] csr_write_data;
    logic csr_read_enable;
    logic csr_write_enable;

    // Privilege Control Loop.
    always_comb
        begin : privilege_control

        end

    // CSR R/W Control Loop.
    always_comb
        begin : csr_rw_control
            // Set default state.
            csr_write_data      = w_data_core_i;
            csr_write_enable    = 1'b1;
            csr_read_enable     = 1'b1;

            if (privilege_violation_w)
                begin
                    csr_read_enable     = 1'b0;
                    csr_write_enable    = 1'b0;
                end

            unique case (rw_cmd_i)
                4'b0001: csr_wdata = w_data_core_i;                // Write and Read
                4'b0010: csr_wdata = w_data_core_i | csr_rdata;    // Set and Read
                4'b0011: csr_wdata = (~w_data_core_i) & csr_rdata; // Clear and Read
                4'b0101: csr_we    = 1'b0;                         // Read only
                4'b1000: csr_read  = 1'b0;                         // Write only
                default:
                    begin
                        csr_we   = 1'b0;
                        csr_read = 1'b0;
                    end
            endcase
        end

    // CSR Logic Loop.
    always_comb
        begin : csr_logic
            // Drive mstatus.
            // Note: There are 3 possible states:
            // 1) Default.
            // 2) In-Interrupt (An interrupt is caught, and handled).
            // 2) Post-Interrupt (An interrupt has already been served)

            // In-interrupt state.
            mstatus_w.fs = OFF;
            mstatus_w.vs = OFF;
        end

    // Driving Loop.
    always_ff @ (posedge clock_i, negedge reset_ni)
        if (!reset_ni) 
            begin
                privilege_level_r   = MACHINE;
               
                mstatus_r           = '0;
                misa_r              = '0;
                mtvec_r             = '0;
                medeleg_r           = '0;
                mideleg_r           = '0;
                mie_r               = '0;
                mtvec_r             = '0;
                mcounteren_r        = '0;

                mscratch_r          = '0; 
                mepc_r              = '0; 
                mcause_r            = '0; 
                mtval_r             = '0; 
                mip_r               = '0; 
                mtinst_r            = '0; 
                mtval2_r            = '0; 

                sstatus_r           = '0;
                sie_r               = '0;
                stvec_r             = '0;
                scounteren_r        = '0;

                sscratch_r          = '0;
                sepc_r              = '0;
                scause_r            = '0;
                stval_r             = '0;
                sip_r               = '0;
                scountovf_r         = '0;
            end 
        
        else
            begin
                privilege_level_r   = privilege_level_w;
               
                mstatus_r           = mstatus_w   ;
                misa_r              = misa_w      ;
                mtvec_r             = mtvec_w     ;
                medeleg_r           = medeleg_w   ;
                mideleg_r           = mideleg_w   ;
                mie_r               = mie_w       ;
                mtvec_r             = mtvec_w     ;
                mcounteren_r        = mcounteren_w;

                mscratch_r          = mscratch_w; 
                mepc_r              = mepc_w    ; 
                mcause_r            = mcause_w  ; 
                mtval_r             = mtval_w   ; 
                mip_r               = mip_w     ; 
                mtinst_r            = mtinst_w  ; 
                mtval2_r            = mtval2_w  ; 

                sstatus_r           = sstatus_w   ;
                sie_r               = sie_w       ;
                stvec_r             = stvec_w     ;
                scounteren_r        = scounteren_w;

                sscratch_r          = sscratch_w ;
                sepc_r              = sepc_w     ;
                scause_r            = scause_w   ;
                stval_r             = stval_w    ;
                sip_r               = sip_w      ;
                scountovf_r         = scountovf_w;
            end


endmodule