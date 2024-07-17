/*
 * Copyright 2023 BSC*
 * *Barcelona Supercomputing Center (BSC)
 * 
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 * 
 * Licensed under the Solderpad Hardware License v 2.1 (the “License”); you
 * may not use this file except in compliance with the License, or, at your
 * option, the Apache License version 2.0. You may obtain a copy of the
 * License at
 * 
 * https://solderpad.org/licenses/SHL-2.1/
 * 
 * Unless required by applicable law or agreed to in writing, any work
 * distributed under the License is distributed on an “AS IS” BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

module escher_csr 
    import escher_riscv_pkg::*;
#(
    parameter word_width     = 64,
    parameter csr_addr_width = 12,
    parameter boot_addr      = 'h100,
    parameter ASID_WIDTH     = 0,             //! Specify how many Address Space ID (ASID) bits are being supported
    parameter RETIRE_BW      = 2,
    parameter COUNTER_EVENTS = 4            //! How many events can be connected to each HPM counter
)(
    input logic                             clk_i,
    input logic                             rstn_i,

    input logic [word_width-1:0]            core_id_i,                    // hartid, for multicore systems
    `ifdef PITON_CINCORANCH
    input   logic [1:0]                     boot_main_id_i,             // CINCORANCH Specific boot id 
    `endif  // Custom for CincoRanch

    // RW interface with the core
    input logic [csr_addr_width-1:0]        rw_addr_i,                  //read and write address form the core
    input logic [3:0]                       rw_cmd_i,                   //specific operation to execute from the core 
    input logic [word_width-1:0]            w_data_core_i,              //write data from the core
    output logic [word_width-1:0]           r_data_core_o,              // read data to the core, address specified with the rw_addr_i

    //Exceptions 
    input logic                             ex_i,                       // exception produced in the core
    input logic [word_width-1:0]            ex_cause_i,                 //cause of the exception
    input logic [63:0]                      pc_i,                       //pc were the exception is produced

    input logic [RETIRE_BW-1:0]             retire_i,                   // shows if a instruction is retired from the core.
    
    `ifdef LOX
    `ifdef SIM_COMMIT_LOG_DPI
    input logic                             torture_dpi_we_i,
    `endif
    `endif

    //Interruptions
    input logic                             rocc_interrupt_i,           // interrupt from the Rocc module
    input logic                             time_irq_i,                 // timer interrupt
    input logic                             irq_i,                      // external interrupt in
    input logic                             m_soft_irq_i,               // Machine software interrupt form the axi module
    output logic                            interrupt_o,                // Inerruption wire to the core
    output logic [word_width-1:0]           interrupt_cause_o,          // Interruption cause

    input  logic [word_width-1:0]           time_i,                    // time passed since the core is reset

    //PCR req inputs
    input  logic                            pcr_req_ready_i,            // ready bit of the pcr

    //PCR resp inputs
    input  logic                            pcr_resp_valid_i,           // ready bit of the pcr
    input  logic [word_width-1:0]           pcr_resp_data_i,            // read data from performance counter module
    input  logic                            pcr_resp_core_id_i,         // core id of the tile that the date is sended

    //PCR outputs request
    output logic                            pcr_req_valid_o,            // valid bit to make a pcr request
    output logic  [csr_addr_width-1:0]      pcr_req_addr_o,             // read/write address to performance counter module (up to 29 aux counters possible in riscv encoding.h)
    output logic  [63:0]                    pcr_req_data_o,             // write data to performance counter module
    output logic  [2:0]                     pcr_req_we_o,               // Cmd of the petition
    output logic                            pcr_req_core_id_o,          // core id of the tile

    //PCR update inputs
    input  logic                            pcr_update_valid_i,
    input  logic                            pcr_update_broadcast_i,
    input  logic                            pcr_update_core_id_i,
    input  logic [csr_addr_width-1:0]       pcr_update_addr_i,
    input  logic [word_width-1:0]           pcr_update_data_i,

    // HPM
    input logic  [COUNTER_EVENTS-1:1]       hpm_events_i [HPM_COUNTERS-1:0],

    // floating point flags
    input logic                             fcsr_flags_valid_i,
    input logic [4:0]                       fcsr_flags_bits_i,
    output logic [2:0]                      fcsr_rm_o,
    output logic [1:0]                      fcsr_fs_o,

    output logic [1:0]                      vcsr_vs_o,

    output logic                            csr_replay_o,               // replay send to the core because there are some parts that are bussy
    output logic                            csr_stall_o,                // The csr are waiting a resp and de core is stalled
    output logic                            csr_xcpt_o,                 // Exeption pproduced by the csr   
    output logic [63:0]                     csr_xcpt_cause_o,           // Exception cause
    output logic [63:0]                     csr_tval_o,                 // Value written to the tval registers
    output logic                            eret_o,

    output logic [word_width-1:0]           status_o,                   //actual mstatus of the core
    output logic [1:0]                      priv_lvl_o,                 // actual privialge level of the core
    output logic [1:0]                      ld_st_priv_lvl_o,
    output logic                            en_ld_st_translation_o,
    output logic                            en_translation_o,

    output logic [2:0]                      bke_queues_inorder_o,       // Defines if bke queues issue inorder.
    `ifdef FPGA
    output logic [63:0]                     ila_dbg_cfg_o,              // Control register to configure ILA trigger handler.
    output logic [63:0]                     ila_cnd_pc_o,               // ILA Conditional trigger program counter value.
    output logic [63:0]                     ila_cnd_maddr_o,            // ILA Conditional trigger memory address value.
    output logic [63:0]                     ila_cnd_time_o,             // ILA Conditional trigger TIME register value.
    output logic [63:0]                     ila_cnd_mask_o,             // ILA Conditional register mask value.
    `endif
    output logic [63:0]                     satp_o,                     // SATP Register includes page table base pointer for the PTW. TODO: Unify definitions across CRS_PKG, MMU_PKG and RISCV_PKG


    output logic [63:0]                     evec_o,                      // virtual address of the PC to execute after a Interrupt or exception

    output logic                            flush_o,                    // the core is executing a sfence.vm instruction and a tlb flush is needed
    output logic [39:0]                     vpu_csr_o

    // TODO: delete ports
    // output logic [csr_addr_width-1:0]       perf_addr_o,                // read/write address to performance counter module
    // output logic [63:0]                     perf_data_o,                // write data to performance counter module
    // input  logic [63:0]                     perf_data_i,                // read data from performance counter module
    // output logic                            perf_we_o,
    // output logic [31:0]                     perf_mcountinhibit_o,
    // input  logic                            perf_count_ovf_int_req_i,
    // input  logic [31:3]                     perf_mhpm_ovf_bits_i
);

    localparam int HPM_ID_BITS      = $clog2(HPM_COUNTERS);
    //localparam int MHPM_TO_HPM_DIST = escher_csr_pkg::CSR_HPM_COUNTER_3 - escher_csr_pkg::CSR_MHPM_COUNTER_3;

    //////////////////////////////////////////////
    // Registers declaration
    //////////////////////////////////////////////

    escher_csr_pkg::uarch_cfg_csr_t  uarch_cfg_csr_q,  uarch_cfg_csr_d;
    escher_csr_pkg::status_rv64_t  mstatus_q,  mstatus_d;
    escher_csr_pkg::satp_t         satp_q, satp_d;
    escher_csr_pkg::csr_t  csr_addr;
    // privilege level register
    escher_csr_pkg::priv_lvl_t   priv_lvl_d, priv_lvl_q;

    logic        mtvec_rst_load_q;// used to determine whether we came out of reset
    logic [63:0] mtvec_q,     mtvec_d;
    logic [63:0] medeleg_q,   medeleg_d;
    logic [63:0] mideleg_q,   mideleg_d;
    logic [63:0] mip_q,       mip_d;
    logic [63:0] mie_q,       mie_d;
    logic [31:0] mcounteren_q,mcounteren_d;
    logic [31:0] mcountinhibit_q,mcountinhibit_d;
    logic [63:0] mscratch_q,  mscratch_d;
    logic [63:0] mepc_q,      mepc_d;
    logic [63:0] mcause_q,    mcause_d;
    logic [63:0] mtval_q,     mtval_d;

    `ifdef FPGA
    escher_csr_pkg::ila_dbg_cfg_csr_t ila_dbg_cfg_q, ila_dbg_cfg_d;
    logic [63:0] ila_cnd_pc_q,      ila_cnd_pc_d;
    logic [63:0] ila_cnd_maddr_q,   ila_cnd_maddr_d;
    logic [63:0] ila_cnd_time_q,    ila_cnd_time_d;
    logic [63:0] ila_cnd_mask_q,    ila_cnd_mask_d;

    escher_csr_pkg::ila_dbg_cfg_csr_t ila_dbg_cfg_init;

    assign ila_dbg_cfg_init.reserved    = 'b0;
    assign ila_dbg_cfg_init.ebrk        = 1'b0;
    assign ila_dbg_cfg_init.rdc         = 1'b0;
    assign ila_dbg_cfg_init.tgb         = 1'b0;
    assign ila_dbg_cfg_init.ctime       = 3'b000;
    assign ila_dbg_cfg_init.cmadr       = 3'b000;
    assign ila_dbg_cfg_init.cpc         = 3'b000;
    assign ila_dbg_cfg_init.swtg        = 1'b0;
    assign ila_dbg_cfg_init.swen        = 1'b0;
    assign ila_dbg_cfg_init.hwen        = 1'b1;

    `endif

    logic [63:0] hpm_rdata, hpm_wdata;
    logic hpm_read, hpm_write;
    logic hpm_rd_event, hpm_wr_event;
    logic [HPM_ID_BITS-1:0] hpm_rd_addr, hpm_wr_address;

    logic [63:0] stvec_q,     stvec_d;
    logic [31:0] scounteren_q,scounteren_d;
    logic [63:0] sscratch_q,  sscratch_d;
    logic [63:0] sepc_q,      sepc_d;
    logic [63:0] scause_q,    scause_d;
    logic [63:0] stval_q,     stval_d;

    // Vector extension registers
    logic [63:0] vl_q,        vl_d;
    logic [63:0] vtype_q,     vtype_d;

    logic [63:0] cycle_q,     cycle_d;
 
    logic [63:0] instret_q,   instret_d;
    logic [31:0] scountovf_q, scountovf_d;

    escher_csr_pkg::fcsr_t fcsr_q, fcsr_d;

    //////////////////////////////////////////////
    // Intermidiet wires and regs declaration
    //////////////////////////////////////////////

    // internal signal to keep track of access exceptions
    logic        read_access_exception, update_access_exception, update_access_exception_vs, privilege_violation;
    logic        csr_we, csr_read;
    logic        csr_xcpt;          // Internal csr exception bit.
    logic [63:0] csr_xcpt_cause;    // cause of the internal csr exception
    logic [63:0] ex_tval;
    logic [63:0] csr_wdata, csr_rdata;
    logic [63:0] trap_vector_base;
    escher_csr_pkg::priv_lvl_t   trap_to_priv_lvl;
    // register for enabling load store address translation, this is critical, hence the register
    logic        en_ld_st_translation_d, en_ld_st_translation_q;
    logic  mprv;
    logic  mret;  // return from M-mode exception
    logic  sret;  // return from S-mode exception
    // CSR write causes us to mark the FPU state as dirty
    logic  dirty_fp_state_csr;
    logic  dirty_v_state_csr;

    // actual state wires
    logic system_insn;
    logic priv_sufficient;
    logic wfi_d, wfi_q;

    // instruction wires
    logic insn_call;
    logic insn_break;
    logic insn_mret;
    logic insn_ret;
    logic insn_sret;
    logic insn_sfence_vm; 
    logic insn_wfi;

    // vector instruction wires
    logic vsetvl_insn;
    logic [10:0] vtype_new;
    logic [63:0] vlmax;

    // pcr wires
    logic pcr_wait_resp_d, pcr_wait_resp_q; // the csr regfile is waiting the response of the PCR
    logic pcr_req_valid; // the csr regfile requests data to the PCR
    logic [63:0] reg_time_d, reg_time_q; // time value from de PCR
    logic cpu_ren; // is needed a read to the csr (write, read, set and clear)
    logic pcr_addr_valid; // the address requested is a pcr address.

    //interruption wires
    logic cond_m_int, cond_s_int;
    logic [63:0] interrupt_cause_q, interrupt_cause_d, interrupt_cause;
    logic interrupt_d, interrupt_q;
    logic global_enable;

    // flush caused by a sfence instruction
    logic flush_sfence;
    
    //logic [11:0] perf_addr_dist;


    //////////////////////////////////////////////
    // System Instructions Decode
    //////////////////////////////////////////////
    // indicates if the request is a system instuction
    assign system_insn = (rw_cmd_i == 4'b0100) ? 1'b1 : 1'b0;
    assign priv_sufficient = priv_lvl_q >= rw_addr_i[9:8];
    // the instructions are codified using the rw_addr_i
    assign insn_call        = ((10'b0000000000 == rw_addr_i[9:0])   && system_insn)                    ? 1'b1 : 1'b0;
    assign insn_break       = ((10'b0000000001 == rw_addr_i[9:0])   && system_insn)                    ? 1'b1 : 1'b0;
    assign insn_mret        = ((10'b1100000010 == rw_addr_i[9:0])   && system_insn && priv_sufficient) ? 1'b1 : 1'b0;
    assign insn_sret        = ((10'b0100000010 == rw_addr_i[9:0])   && system_insn && priv_sufficient) ? 1'b1 : 1'b0;
    assign insn_sfence_vm   = ((5'b01001 == rw_addr_i[9:5])         && system_insn && priv_sufficient) ? 1'b1 : 1'b0;
    assign insn_wfi         = ((10'b0100000101 == rw_addr_i[9:0])   && system_insn && priv_sufficient) ? 1'b1 : 1'b0;

    //////////////////////////////////////////////
    // Vector Instructions Decode
    //////////////////////////////////////////////
    assign vsetvl_insn = ((rw_cmd_i == 4'b0110) || (rw_cmd_i == 4'b0111)) ? 1'b1 : 1'b0;

    //////////////////////////////////////////////
    // VPU Instructions Decode
    //////////////////////////////////////////////
    // indicates if the request is a system instuction

    // ----------------
    // Assignments
    // ----------------
    assign csr_addr = escher_csr_pkg::csr_t'(rw_addr_i);

    // ----------------
    // HPM Assignments
    // ----------------

    // assign perf_mcountinhibit_o = mcountinhibit_q;
    // assign perf_addr_o = csr_addr.address[11:0] - perf_addr_dist;

    // ----------------
    // CSR Read logic
    // ----------------
    always_comb begin : csr_read_process
        // a read access exception can only occur if we attempt to read a CSR which does not exist
        read_access_exception = 1'b0;
        pcr_addr_valid = 1'b0;
        csr_rdata = 64'b0;
        hpm_read = 1'b0;
        hpm_rd_event = 1'b0;
        hpm_rd_addr = '0;
        //perf_addr_dist = '0;


        if (csr_read) begin
            unique case (csr_addr.address)
                escher_csr_pkg::CSR_FFLAGS: begin
                    if (mstatus_q.fs == escher_csr_pkg::Off_State) begin
                        read_access_exception = 1'b1;
                    end else begin
                        csr_rdata = {59'b0, fcsr_q.fflags};
                    end
                end
                escher_csr_pkg::CSR_FRM: begin
                    if (mstatus_q.fs == escher_csr_pkg::Off_State) begin
                        read_access_exception = 1'b1;
                    end else begin
                        csr_rdata = {61'b0, fcsr_q.frm};
                    end
                end
            `ifdef LAGARTO_KA //VPU
                escher_csr_pkg::CSR_FCSR: begin
                    if (mstatus_q.fs == escher_csr_pkg::Off_State) begin
                        read_access_exception = 1'b1;
                    end else begin
                        csr_rdata = {53'b0, fcsr_q.vxrm, fcsr_q.vxsat, fcsr_q.frm, fcsr_q.fflags};
                    end
                end
                escher_csr_pkg::CSR_VXSAT: begin
                    if (mstatus_q.vs == escher_csr_pkg::Off_State) begin
                        read_access_exception = 1'b1;
                    end else begin
                        csr_rdata = {63'b0, fcsr_q.vxsat};
                    end
                end
                escher_csr_pkg::CSR_VXRM: begin
                    if (mstatus_q.vs == escher_csr_pkg::Off_State) begin
                        read_access_exception = 1'b1;
                    end else begin
                        csr_rdata = {62'b0, fcsr_q.vxrm};

                    end
                end
            `else 
                escher_csr_pkg::CSR_FCSR: begin
                    if (mstatus_q.fs == escher_csr_pkg::Off_State) begin
                        read_access_exception = 1'b1;
                    end else begin
                        csr_rdata = {56'b0, fcsr_q.frm, fcsr_q.fflags};
                    end
                end
            `endif
                escher_csr_pkg::CSR_VL: begin
                    if (mstatus_q.vs == escher_csr_pkg::Off_State) begin
                        read_access_exception = 1'b1;
                    end else begin
                        csr_rdata = vl_q;
                    end
                end

                escher_csr_pkg::CSR_VTYPE: begin
                    if (mstatus_q.vs == escher_csr_pkg::Off_State) begin
                        read_access_exception = 1'b1;
                    end else begin
                        csr_rdata = vtype_q;
                    end
                end

                // uArch config
                escher_csr_pkg::CSR_UARCH_CFG:          csr_rdata = uarch_cfg_csr_q;

                // ILA config
                `ifdef FPGA
                escher_csr_pkg::CSR_ILA_DBG_CFG:        csr_rdata = ila_dbg_cfg_q;
                escher_csr_pkg::CSR_ILA_CND_PC:         csr_rdata = ila_cnd_pc_q;
                escher_csr_pkg::CSR_ILA_CND_MADDR:      csr_rdata = ila_cnd_maddr_q;
                escher_csr_pkg::CSR_ILA_CND_TIME:       csr_rdata = ila_cnd_time_q;
                escher_csr_pkg::CSR_ILA_CND_MASK:       csr_rdata = ila_cnd_mask_q;
                `endif

                // debug registers
                escher_csr_pkg::CSR_DCSR:               csr_rdata = 64'b0; // not implemented
                escher_csr_pkg::CSR_DPC:                csr_rdata = 64'b0; // not implemented
                escher_csr_pkg::CSR_DSCRATCH0:          csr_rdata = 64'b0; // not implemented
                escher_csr_pkg::CSR_DSCRATCH1:          csr_rdata = 64'b0; // not implemented
                // trigger module registers
                escher_csr_pkg::CSR_TSELECT:; // not implemented
                escher_csr_pkg::CSR_TDATA1:;  // not implemented
                escher_csr_pkg::CSR_TDATA2:;  // not implemented
                escher_csr_pkg::CSR_TDATA3:;  // not implemented
                // supervisor registers
                escher_csr_pkg::CSR_SSTATUS: begin
                    csr_rdata = mstatus_q & escher_csr_pkg::SMODE_STATUS_READ_MASK;
                end
                escher_csr_pkg::CSR_SIE:                csr_rdata = mie_q & mideleg_q;
                escher_csr_pkg::CSR_SIP:                csr_rdata = mip_q & mideleg_q;
                escher_csr_pkg::CSR_STVEC:              csr_rdata = stvec_q;
                escher_csr_pkg::CSR_SCOUNTEREN:         csr_rdata = {{32{1'b0}}, scounteren_q};
                escher_csr_pkg::CSR_SSCRATCH:           csr_rdata = sscratch_q;
                escher_csr_pkg::CSR_SEPC:               csr_rdata = sepc_q;
                escher_csr_pkg::CSR_SCAUSE:             csr_rdata = scause_q;
                escher_csr_pkg::CSR_STVAL:              csr_rdata = stval_q;
                escher_csr_pkg::CSR_SATP: begin
                    // intercept reads to SATP if in S-Mode and TVM is enabled
                    if ((priv_lvl_o == escher_csr_pkg::PRIV_LVL_S) && mstatus_q.tvm) begin
                        read_access_exception = 1'b1;
                    end else begin
                        csr_rdata = satp_q;
                    end
                end
                // machine mode registers
                escher_csr_pkg::CSR_MSTATUS:            csr_rdata = mstatus_q;
                escher_csr_pkg::CSR_MISA:               csr_rdata = escher_csr_pkg::ISA_CODE; 
                escher_csr_pkg::CSR_MEDELEG:            csr_rdata = medeleg_q;
                escher_csr_pkg::CSR_MIDELEG:            csr_rdata = mideleg_q;
                escher_csr_pkg::CSR_MIE:                csr_rdata = mie_q;
                escher_csr_pkg::CSR_MTVEC:              csr_rdata = mtvec_q;
                escher_csr_pkg::CSR_MCOUNTEREN:         csr_rdata = {{32{1'b0}}, mcounteren_q};
                escher_csr_pkg::CSR_MSCRATCH:           csr_rdata = mscratch_q;
                escher_csr_pkg::CSR_MEPC:               csr_rdata = mepc_q;
                escher_csr_pkg::CSR_MCAUSE:             csr_rdata = mcause_q;
                escher_csr_pkg::CSR_MTVAL:              csr_rdata = mtval_q;
                escher_csr_pkg::CSR_MIP:                csr_rdata = mip_q;
                escher_csr_pkg::CSR_MVENDORID:          csr_rdata = 64'b0; // not implemented
                escher_csr_pkg::CSR_MARCHID:            csr_rdata = 64'b0; // not implemented
                escher_csr_pkg::CSR_MIMPID:             csr_rdata = 64'b0; // not implemented
                escher_csr_pkg::CSR_MHARTID:            csr_rdata = core_id_i; 
                `ifdef PITON_CINCORANCH
                escher_csr_pkg::CSR_MBOOT_MAIN_ID:      csr_rdata = {62'b0, boot_main_id_i};
                `endif  // Custom for CincoRanch

                // Counters and Timers
                escher_csr_pkg::CSR_MCYCLE:             csr_rdata = cycle_q;
                escher_csr_pkg::CSR_MINSTRET:           csr_rdata = instret_q;
                escher_csr_pkg::CSR_MHPM_COUNTER_3,
                escher_csr_pkg::CSR_MHPM_COUNTER_4,
                escher_csr_pkg::CSR_MHPM_COUNTER_5,
                escher_csr_pkg::CSR_MHPM_COUNTER_6,
                escher_csr_pkg::CSR_MHPM_COUNTER_7,
                escher_csr_pkg::CSR_MHPM_COUNTER_8,
                escher_csr_pkg::CSR_MHPM_COUNTER_9,
                escher_csr_pkg::CSR_MHPM_COUNTER_10,
                escher_csr_pkg::CSR_MHPM_COUNTER_11,
                escher_csr_pkg::CSR_MHPM_COUNTER_12,
                escher_csr_pkg::CSR_MHPM_COUNTER_13,
                escher_csr_pkg::CSR_MHPM_COUNTER_14,
                escher_csr_pkg::CSR_MHPM_COUNTER_15,
                escher_csr_pkg::CSR_MHPM_COUNTER_16,
                escher_csr_pkg::CSR_MHPM_COUNTER_17,
                escher_csr_pkg::CSR_MHPM_COUNTER_18,
                escher_csr_pkg::CSR_MHPM_COUNTER_19,
                escher_csr_pkg::CSR_MHPM_COUNTER_20,
                escher_csr_pkg::CSR_MHPM_COUNTER_21,
                escher_csr_pkg::CSR_MHPM_COUNTER_22,
                escher_csr_pkg::CSR_MHPM_COUNTER_23,
                escher_csr_pkg::CSR_MHPM_COUNTER_24,
                escher_csr_pkg::CSR_MHPM_COUNTER_25,
                escher_csr_pkg::CSR_MHPM_COUNTER_26,
                escher_csr_pkg::CSR_MHPM_COUNTER_27,
                escher_csr_pkg::CSR_MHPM_COUNTER_28,
                escher_csr_pkg::CSR_MHPM_COUNTER_29,
                escher_csr_pkg::CSR_MHPM_COUNTER_30,
                escher_csr_pkg::CSR_MHPM_COUNTER_31: begin
                    hpm_rd_addr = (csr_addr.address - escher_csr_pkg::CSR_MHPM_COUNTER_3);
                    hpm_read  = 1'b1;
                    csr_rdata = hpm_rdata;
                end

                escher_csr_pkg::CSR_CYCLE:              csr_rdata = cycle_q;
                escher_csr_pkg::CSR_TIME:               csr_rdata = reg_time_q;
                escher_csr_pkg::CSR_INSTRET:            csr_rdata = instret_q;
                escher_csr_pkg::CSR_HPM_COUNTER_3,
                escher_csr_pkg::CSR_HPM_COUNTER_4,
                escher_csr_pkg::CSR_HPM_COUNTER_5,
                escher_csr_pkg::CSR_HPM_COUNTER_6,
                escher_csr_pkg::CSR_HPM_COUNTER_7,
                escher_csr_pkg::CSR_HPM_COUNTER_8,
                escher_csr_pkg::CSR_HPM_COUNTER_9,
                escher_csr_pkg::CSR_HPM_COUNTER_10,
                escher_csr_pkg::CSR_HPM_COUNTER_11,
                escher_csr_pkg::CSR_HPM_COUNTER_12,
                escher_csr_pkg::CSR_HPM_COUNTER_13,
                escher_csr_pkg::CSR_HPM_COUNTER_14,
                escher_csr_pkg::CSR_HPM_COUNTER_15,
                escher_csr_pkg::CSR_HPM_COUNTER_16,
                escher_csr_pkg::CSR_HPM_COUNTER_17,
                escher_csr_pkg::CSR_HPM_COUNTER_18,
                escher_csr_pkg::CSR_HPM_COUNTER_19,
                escher_csr_pkg::CSR_HPM_COUNTER_20,
                escher_csr_pkg::CSR_HPM_COUNTER_21,
                escher_csr_pkg::CSR_HPM_COUNTER_22,
                escher_csr_pkg::CSR_HPM_COUNTER_23,
                escher_csr_pkg::CSR_HPM_COUNTER_24,
                escher_csr_pkg::CSR_HPM_COUNTER_25,
                escher_csr_pkg::CSR_HPM_COUNTER_26,
                escher_csr_pkg::CSR_HPM_COUNTER_27,
                escher_csr_pkg::CSR_HPM_COUNTER_28,
                escher_csr_pkg::CSR_HPM_COUNTER_29,
                escher_csr_pkg::CSR_HPM_COUNTER_30,
                escher_csr_pkg::CSR_HPM_COUNTER_31: begin
                    hpm_rd_addr     = (csr_addr.address - escher_csr_pkg::CSR_HPM_COUNTER_3);
                    hpm_read        = 1'b1;
                    hpm_rd_event    = 1'b1;
                    csr_rdata       = hpm_rdata;
                end
                escher_csr_pkg::CSR_SCOUNTOVF:         csr_rdata = {{32{1'b0}}, scountovf_q & mcounteren_q};
                escher_csr_pkg::CSR_MHPM_EVENT_3,
                escher_csr_pkg::CSR_MHPM_EVENT_4,
                escher_csr_pkg::CSR_MHPM_EVENT_5,
                escher_csr_pkg::CSR_MHPM_EVENT_6,
                escher_csr_pkg::CSR_MHPM_EVENT_7,
                escher_csr_pkg::CSR_MHPM_EVENT_8,
                escher_csr_pkg::CSR_MHPM_EVENT_9,
                escher_csr_pkg::CSR_MHPM_EVENT_10,
                escher_csr_pkg::CSR_MHPM_EVENT_11,
                escher_csr_pkg::CSR_MHPM_EVENT_12,
                escher_csr_pkg::CSR_MHPM_EVENT_13,
                escher_csr_pkg::CSR_MHPM_EVENT_14,
                escher_csr_pkg::CSR_MHPM_EVENT_15,
                escher_csr_pkg::CSR_MHPM_EVENT_16,
                escher_csr_pkg::CSR_MHPM_EVENT_17,
                escher_csr_pkg::CSR_MHPM_EVENT_18,
                escher_csr_pkg::CSR_MHPM_EVENT_19,
                escher_csr_pkg::CSR_MHPM_EVENT_20,
                escher_csr_pkg::CSR_MHPM_EVENT_21,
                escher_csr_pkg::CSR_MHPM_EVENT_22,
                escher_csr_pkg::CSR_MHPM_EVENT_23,
                escher_csr_pkg::CSR_MHPM_EVENT_24,
                escher_csr_pkg::CSR_MHPM_EVENT_25,
                escher_csr_pkg::CSR_MHPM_EVENT_26,
                escher_csr_pkg::CSR_MHPM_EVENT_27,
                escher_csr_pkg::CSR_MHPM_EVENT_28,
                escher_csr_pkg::CSR_MHPM_EVENT_29,
                escher_csr_pkg::CSR_MHPM_EVENT_30,
                escher_csr_pkg::CSR_MHPM_EVENT_31:   begin
                    hpm_rd_addr = (csr_addr.address - escher_csr_pkg::CSR_MHPM_EVENT_3);
                    hpm_read  = 1'b1;
                    csr_rdata = hpm_rdata;
                end

                escher_csr_pkg::CSR_MEM_MAP_0,
                escher_csr_pkg::CSR_MEM_MAP_1,
                escher_csr_pkg::CSR_MEM_MAP_2,
                escher_csr_pkg::CSR_MEM_MAP_3,
                escher_csr_pkg::CSR_MEM_MAP_4,
                escher_csr_pkg::CSR_MEM_MAP_5,
                escher_csr_pkg::CSR_MEM_MAP_6,
                escher_csr_pkg::CSR_MEM_MAP_7,
                escher_csr_pkg::CSR_MEM_MAP_8,
                escher_csr_pkg::CSR_MEM_MAP_9,
                escher_csr_pkg::CSR_MEM_MAP_10,                
                escher_csr_pkg::CSR_MEM_MAP_11,                
                escher_csr_pkg::CSR_MEM_MAP_12,                
                escher_csr_pkg::CSR_MEM_MAP_13,                
                escher_csr_pkg::CSR_MEM_MAP_14,                
                escher_csr_pkg::CSR_MEM_MAP_15,
                escher_csr_pkg::CSR_IO_MAP_0,
                escher_csr_pkg::CSR_IO_MAP_1,
                escher_csr_pkg::CSR_IO_MAP_2,
                escher_csr_pkg::CSR_IO_MAP_3,
                escher_csr_pkg::CSR_IO_MAP_4,
                escher_csr_pkg::CSR_IO_MAP_5,
                escher_csr_pkg::CSR_IO_MAP_6,
                escher_csr_pkg::CSR_IO_MAP_7,
                escher_csr_pkg::CSR_IO_MAP_8,
                escher_csr_pkg::CSR_IO_MAP_9,
                escher_csr_pkg::CSR_IO_MAP_10,
                escher_csr_pkg::CSR_IO_MAP_11,
                escher_csr_pkg::CSR_IO_MAP_12,
                escher_csr_pkg::CSR_IO_MAP_13,
                escher_csr_pkg::CSR_IO_MAP_14,
                escher_csr_pkg::CSR_IO_MAP_15,
                escher_csr_pkg::CSR_IRQ_MAP_0,
                escher_csr_pkg::CSR_IRQ_MAP_1,
                escher_csr_pkg::CSR_IRQ_MAP_2,
                escher_csr_pkg::CSR_IRQ_MAP_3,
                escher_csr_pkg::CSR_IRQ_MAP_4,
                escher_csr_pkg::CSR_IRQ_MAP_5,
                escher_csr_pkg::CSR_IRQ_MAP_6,
                escher_csr_pkg::CSR_IRQ_MAP_7,
                escher_csr_pkg::CSR_IRQ_MAP_8,
                escher_csr_pkg::CSR_IRQ_MAP_9,
                escher_csr_pkg::CSR_IRQ_MAP_10,
                escher_csr_pkg::CSR_IRQ_MAP_11,
                escher_csr_pkg::CSR_IRQ_MAP_12,
                escher_csr_pkg::CSR_IRQ_MAP_13,
                escher_csr_pkg::CSR_IRQ_MAP_14,
                escher_csr_pkg::CSR_IRQ_MAP_15,
                escher_csr_pkg::FROM_HOST,
                escher_csr_pkg::CSR_HYPERRAM_CONFIG, 
                escher_csr_pkg::CSR_SPI_CONFIG, 
                escher_csr_pkg::CSR_CNM_CONFIG,
                escher_csr_pkg::TO_HOST: begin            
                                        csr_rdata = pcr_resp_data_i;
                                        pcr_addr_valid = 1'b1;
                end
                
                escher_csr_pkg::CSR_PMPCFG_0:;
                escher_csr_pkg::CSR_PMPCFG_1:;
                escher_csr_pkg::CSR_PMPCFG_2:;
                escher_csr_pkg::CSR_PMPCFG_3:;

                escher_csr_pkg::CSR_PMPADDR_0:;
                escher_csr_pkg::CSR_PMPADDR_1:;
                escher_csr_pkg::CSR_PMPADDR_2:;
                escher_csr_pkg::CSR_PMPADDR_3:;
                escher_csr_pkg::CSR_PMPADDR_4:;
                escher_csr_pkg::CSR_PMPADDR_5:;
                escher_csr_pkg::CSR_PMPADDR_6:;
                escher_csr_pkg::CSR_PMPADDR_7:;
                escher_csr_pkg::CSR_PMPADDR_8:;
                escher_csr_pkg::CSR_PMPADDR_9:;
                escher_csr_pkg::CSR_PMPADDR_10:;
                escher_csr_pkg::CSR_PMPADDR_11:;
                escher_csr_pkg::CSR_PMPADDR_12:;
                escher_csr_pkg::CSR_PMPADDR_13:;
                escher_csr_pkg::CSR_PMPADDR_14:;
                escher_csr_pkg::CSR_PMPADDR_15:;
                //
                default: read_access_exception = 1'b1;
            endcase
        end
    end


    logic [$clog2(RETIRE_BW+1)-1:0] retire_cnt;
    logic csr_wdata_en;


    assign cycle_d = (!mcountinhibit_q[0]) ? cycle_q + 64'd1 : (csr_wdata_en) ? csr_wdata : cycle_q;
    // ---------------------------
    // CSR Write and update logic
    // ---------------------------
    logic [63:0] mask;

    
    always_comb begin : csr_update
        automatic escher_csr_pkg::satp_t sapt,sapt_temp; // temporal values to correct the structure of the writing on the sapt reg
        automatic logic [63:0] instret;
        automatic logic [63:0] mstatus_int, mstatus_clear ,mstatus_set; //Used to set and clear some bits of the mstatus_d
        automatic logic flush; //temporal flush value befor the exceptions logic
        automatic logic [63:0] mcause_int; // temporal value of mcause
        automatic logic [63:0] scause_int; // temporal value of scause
        automatic logic [63:0] mtval_int; // temporal value of mtval
        automatic logic [63:0] stval_int; // temporal value of stval
        automatic logic [63:0] mepc_int; // temporal value of mepc
        automatic logic [63:0] sepc_int; // temporal value of sepc
        
        
        sapt = satp_q;
        instret = instret_q;

        //Avoid latch
        mcountinhibit_d = {32'b0};
        mask = 64'b0;
        sapt_temp = escher_csr_pkg::satp_t'(0);
        // --------------------
        // Counters
        // --------------------
        // increase instruction retired counter

        retire_cnt = {$clog2(RETIRE_BW+1){1'b0}};
        for (int i=0; i<RETIRE_BW; i++) begin
            retire_cnt += retire_i[i];
        end

        if (!ex_i && ~mcountinhibit_q[2])  begin 
            instret = instret + retire_cnt;
        end
        instret_d = instret;
        
        // increment the cycle count
        // if (!mcountinhibit_q[0]) begin
        // // cycle_d = cycle_q + 1'(~mcountinhibit_q[0]);
        //     cycle_d = cycle_q + 64'd1;
        // end else begin
        //     cycle_d = cycle_q;
        // end
        //scountovf_d = {perf_mhpm_ovf_bits_i, 3'b000};
        scountovf_d = 32'd0;


        eret_o                  = 1'b0;
        flush                   = 1'b0;
        update_access_exception = 1'b0;

        hpm_write               = 1'b0;
        hpm_wdata               = 'b0;
        hpm_wr_event            = 1'b0;
        hpm_wr_address          = '0;
        
        fcsr_d                  = fcsr_q;

        uarch_cfg_csr_d         = uarch_cfg_csr_q;

        `ifdef FPGA
        ila_dbg_cfg_d           = ila_dbg_cfg_q;
        ila_cnd_pc_d            = ila_cnd_pc_q;
        ila_cnd_maddr_d         = ila_cnd_maddr_q;
        ila_cnd_time_d          = ila_cnd_time_q;
        ila_cnd_mask_d          = ila_cnd_mask_q;
        `endif

        priv_lvl_d              = priv_lvl_q;

        mstatus_clear = escher_csr_pkg::MSTATUS_UXL | escher_csr_pkg::MSTATUS_SXL | escher_csr_pkg::MSTATUS64_SD;
        mstatus_set =   (((mstatus_q.xs == escher_csr_pkg::Dirty_State) | (mstatus_q.fs == escher_csr_pkg::Dirty_State) | (mstatus_q.vs == escher_csr_pkg::Dirty_State))<<63) |
                        (escher_csr_pkg::XLEN_64 << 32)|
                        (escher_csr_pkg::XLEN_64 << 34);
        mstatus_int   = (mstatus_q & ~mstatus_clear)| mstatus_set;

        // hardwired extension registers

        // write the floating point status register
        if (fcsr_flags_valid_i) begin
            fcsr_d.fflags = fcsr_flags_bits_i | fcsr_q.fflags;
        end
       

        // check whether we come out of reset
        // this is a workaround. some tools have issues
        // having boot_addr_i in the asynchronous
        // reset assignment to mtvec_d, even though
        // boot_addr_i will be assigned a constant
        // on the top-level.
        if (mtvec_rst_load_q) begin
            mtvec_d             = boot_addr + 'h40;
        end else begin
            mtvec_d             = mtvec_q;
        end

        // ---------------------
        // External Interrupts
        // ---------------------
        // the IRQ_M_EXT = irq_i || rocc_interrupt_i, IRQ_M_SOFT = m_soft_irq_i and IRQ_M_TIMER = time_irq_i
        //mip_d = {mip_q[63:14], perf_count_ovf_int_req_i || mip_q[13], mip_q[12], irq_i || rocc_interrupt_i, mip_q[10:8], time_irq_i, mip_q[6:4], m_soft_irq_i, mip_q[2:0]};
        mip_d = {mip_q[63:14], mip_q[13], mip_q[12], irq_i || rocc_interrupt_i, mip_q[10:8], time_irq_i, mip_q[6:4], m_soft_irq_i, mip_q[2:0]};


        

        medeleg_d               = medeleg_q;
        mideleg_d               = mideleg_q;
        
        mie_d                   = mie_q;
        mepc_int                = mepc_q;
        mcause_int              = mcause_q;
        mcounteren_d            = mcounteren_q;
        mcountinhibit_d         = mcountinhibit_q;
        mscratch_d              = mscratch_q;
        mtval_int               = mtval_q;

        sepc_int                = sepc_q;
        scause_int              = scause_q;
        stvec_d                 = stvec_q;
        scounteren_d            = scounteren_q;
        sscratch_d              = sscratch_q;
        stval_int               = stval_q;
        satp_d                  = satp_q;

        en_ld_st_translation_d  = en_ld_st_translation_q;
        dirty_fp_state_csr      = 1'b0;
        pcr_req_data_o          = 'b0;
        csr_wdata_en            = '0;

        // check for correct access rights and that we are writing
        if (csr_we) begin
            unique case (csr_addr.address)
                // Floating-Point
                escher_csr_pkg::CSR_FFLAGS: begin
                    if (mstatus_q.fs == escher_csr_pkg::Off_State) begin
                        update_access_exception = 1'b1;
                    end else begin
                        dirty_fp_state_csr = 1'b1;
                        fcsr_d.fflags = csr_wdata[4:0];
                        // this instruction has side-effects
                        flush = 1'b1;
                    end
                end
                escher_csr_pkg::CSR_FRM: begin
                    if (mstatus_q.fs == escher_csr_pkg::Off_State) begin
                        update_access_exception = 1'b1;
                    end else begin
                        dirty_fp_state_csr = 1'b1;
                        fcsr_d.frm    = csr_wdata[2:0];
                        // this instruction has side-effects
                        flush = 1'b1;
                    end
                end
                `ifdef LAGARTO_KA //VPU
                escher_csr_pkg::CSR_FCSR: begin
                    if (mstatus_q.fs == escher_csr_pkg::Off_State) begin
                        update_access_exception = 1'b1;
                    end else begin
                        dirty_fp_state_csr = 1'b1;
                        fcsr_d[7:0] = csr_wdata[7:0]; // ignore writes to reserved space
                        fcsr_d.vxsat = csr_wdata[8];
                        fcsr_d.vxrm = csr_wdata[10:9];
                        // this instruction has side-effects
                        flush = 1'b1;
                    end
                end
                escher_csr_pkg::CSR_VXSAT: begin
                    if (mstatus_q.vs == escher_csr_pkg::Off_State) begin
                        update_access_exception = 1'b1;
                    end else begin
                        dirty_fp_state_csr = 1'b1;
                        dirty_v_state_csr  = 1'b1;
                        fcsr_d.vxsat    = csr_wdata[0];
                        // this instruction has side-effects
                        flush = 1'b1;
                    end
                end
                escher_csr_pkg::CSR_VXRM: begin
                    if (mstatus_q.vs == escher_csr_pkg::Off_State) begin
                        update_access_exception = 1'b1;
                    end else begin
                        dirty_fp_state_csr = 1'b1;
                        dirty_v_state_csr  = 1'b1;
                        fcsr_d.vxrm    = csr_wdata[1:0];
                        // this instruction has side-effects
                        flush = 1'b1;
                    end
                end
                `else
                escher_csr_pkg::CSR_FCSR: begin
                    if (mstatus_q.fs == escher_csr_pkg::Off_State) begin
                        update_access_exception = 1'b1;
                    end else begin
                        dirty_fp_state_csr = 1'b1;
                        fcsr_d[7:0] = csr_wdata[7:0]; // ignore writes to reserved space
                        /*vxsat_d = csr_wdata[8];
                        vxrm_d = csr_wdata[10:9];*/
                        // this instruction has side-effects
                        flush = 1'b1;
                    end
                end
                `endif

                // uArch config
                escher_csr_pkg::CSR_UARCH_CFG:     uarch_cfg_csr_d = csr_wdata;

                // ILA config
                `ifdef FPGA
                escher_csr_pkg::CSR_ILA_DBG_CFG:        ila_dbg_cfg_d       = csr_wdata;
                escher_csr_pkg::CSR_ILA_CND_PC:         ila_cnd_pc_d        = csr_wdata;
                escher_csr_pkg::CSR_ILA_CND_MADDR:      ila_cnd_maddr_d     = csr_wdata;
                escher_csr_pkg::CSR_ILA_CND_TIME:       ila_cnd_time_d      = csr_wdata;
                escher_csr_pkg::CSR_ILA_CND_MASK:       ila_cnd_mask_d      = csr_wdata;
                `endif

                // debug CSR
                escher_csr_pkg::CSR_DCSR:;// not implemented
                escher_csr_pkg::CSR_DPC:;// not implemented
                escher_csr_pkg::CSR_DSCRATCH0:;// not implemented
                escher_csr_pkg::CSR_DSCRATCH1:;// not implemented
                // trigger module CSRs
                escher_csr_pkg::CSR_TSELECT:; // not implemented
                escher_csr_pkg::CSR_TDATA1:;  // not implemented
                escher_csr_pkg::CSR_TDATA2:;  // not implemented
                escher_csr_pkg::CSR_TDATA3:;  // not implemented
                // sstatus is a subset of mstatus - mask it accordingly
                escher_csr_pkg::CSR_SSTATUS: begin
                    mask = escher_csr_pkg::SMODE_STATUS_WRITE_MASK;
                    mstatus_int = (mstatus_q & ~mask) | (csr_wdata & mask);
                    // this instruction has side-effects
                    flush = 1'b1;
                end
                // even machine mode interrupts can be visible and set-able to supervisor
                // if the corresponding bit in mideleg is set
                escher_csr_pkg::CSR_SIE: begin
                    // the mideleg makes sure only delegate-able register (and therefore also only implemented registers) are written
                    mask = escher_csr_pkg::MIP_SSIP | escher_csr_pkg::MIP_STIP | escher_csr_pkg::MIP_SEIP | escher_csr_pkg::MIP_LCOFIP;
                    mie_d = (mie_q & ~mask) | (csr_wdata & mask);
                end

                escher_csr_pkg::CSR_SIP: begin
                    // only the supervisor software interrupt is write-able, iff delegated
                    mask = (escher_csr_pkg::MIP_SSIP | escher_csr_pkg::MIP_LCOFIP) & mideleg_q;
                    mip_d = (mip_q & ~mask) | (csr_wdata & mask);
                end

                escher_csr_pkg::CSR_SCOUNTEREN:         scounteren_d = csr_wdata[31:0];
                escher_csr_pkg::CSR_STVEC:              stvec_d     = {csr_wdata[63:2], 1'b0, csr_wdata[0]};
                escher_csr_pkg::CSR_SSCRATCH:           sscratch_d  = csr_wdata;
                escher_csr_pkg::CSR_SEPC:               sepc_int    = {csr_wdata[63:1], 1'b0};
                escher_csr_pkg::CSR_SCAUSE:             scause_int  = csr_wdata;
                escher_csr_pkg::CSR_STVAL:              stval_int   = csr_wdata;
                // supervisor address translation and protection
                escher_csr_pkg::CSR_SATP: begin
                    // intercept SATP writes if in S-Mode and TVM is enabled
                    if ((priv_lvl_o == escher_csr_pkg::PRIV_LVL_S) && mstatus_q.tvm)
                        update_access_exception = 1'b1;
                    else begin
                        sapt_temp      = escher_csr_pkg::satp_t'(csr_wdata);
                        // only make ASID_LEN - 1 bit stick, that way software can figure out how many ASID bits are supported
                        sapt.asid = sapt_temp.asid & {{(16-ASID_WIDTH){1'b0}}, {ASID_WIDTH{1'b1}}};
                        sapt.mode = sapt_temp.mode;
                        sapt.ppn = sapt_temp.ppn;
                        // only update if we actually support this mode
                        if ((sapt.mode == escher_csr_pkg::MODE_OFF) || (sapt.mode == escher_csr_pkg::MODE_SV39)) satp_d = sapt;
                    end
                    // changing the mode can have side-effects on address translation (e.g.: other instructions), re-fetch
                    // the next instruction by executing a flush
                    flush = 1'b1;
                end

                escher_csr_pkg::CSR_MSTATUS: begin
                    // mask of the bits that are set to zero
                    mask = escher_csr_pkg::MSTATUS_UIE | escher_csr_pkg::MSTATUS_UPIE | escher_csr_pkg::MSTATUS_XS | escher_csr_pkg::MSTATUS64_WPRI;
                    mstatus_int      = csr_wdata & ~mask;
                    
                    // this register has side-effects on other registers, flush the pipeline
                    flush        = 1'b1;
                end
                // MISA is WARL (Write Any Value, Reads Legal Value)
                escher_csr_pkg::CSR_MISA:;
                // machine exception delegation register
                // 0 - 15 exceptions supported
                escher_csr_pkg::CSR_MEDELEG: begin
                    mask = (1 << escher_csr_pkg::INSTR_ADDR_MISALIGNED) |
                           (1 << escher_csr_pkg::BREAKPOINT) |
                           (1 << escher_csr_pkg::USER_ECALL) |
                           (1 << escher_csr_pkg::INSTR_PAGE_FAULT) |
                           (1 << escher_csr_pkg::LD_PAGE_FAULT) |
                           (1 << escher_csr_pkg::ST_AMO_PAGE_FAULT);
                    medeleg_d = (medeleg_q & ~mask) | (csr_wdata & mask);
                end
                // machine interrupt delegation register
                // we do not support user interrupt delegation
                escher_csr_pkg::CSR_MIDELEG: begin
                    mask = escher_csr_pkg::MIP_SSIP | escher_csr_pkg::MIP_STIP | escher_csr_pkg::MIP_SEIP | escher_csr_pkg::MIP_LCOFIP;
                    mideleg_d = (mideleg_q & ~mask) | (csr_wdata & mask);
                end
                // mask the register so that unsupported interrupts can never be set
                escher_csr_pkg::CSR_MIE: begin
                    mask = escher_csr_pkg::MIP_SSIP | escher_csr_pkg::MIP_STIP | escher_csr_pkg::MIP_SEIP | escher_csr_pkg::MIP_MSIP | escher_csr_pkg::MIP_MTIP | escher_csr_pkg::MIP_MEIP | escher_csr_pkg::MIP_LCOFIP;
                    mie_d = (mie_q & ~mask) | (csr_wdata & mask); // we only support supervisor and M-mode interrupts
                end

                escher_csr_pkg::CSR_MTVEC: begin
                    mtvec_d = {csr_wdata[63:2], 1'b0, csr_wdata[0]};
                    // we are in vector mode, this implementation requires the additional
                    // alignment constraint of 64 * 4 bytes
                    if (csr_wdata[0]) mtvec_d = {csr_wdata[63:8], 7'b0, csr_wdata[0]};
                end

                escher_csr_pkg::CSR_MCOUNTEREN:         mcounteren_d = csr_wdata[31:0];
                escher_csr_pkg::CSR_MCOUNTINHIBIT:      mcountinhibit_d = csr_wdata[31:0];
                escher_csr_pkg::CSR_MSCRATCH:           mscratch_d  = csr_wdata;
                escher_csr_pkg::CSR_MEPC:               mepc_int    = {csr_wdata[63:1], 1'b0};
                escher_csr_pkg::CSR_MCAUSE:             mcause_int  = csr_wdata;
                escher_csr_pkg::CSR_MTVAL:              mtval_int   = csr_wdata;
                escher_csr_pkg::CSR_MIP: begin
                    mask = escher_csr_pkg::MIP_SSIP | escher_csr_pkg::MIP_STIP | escher_csr_pkg::MIP_SEIP | escher_csr_pkg::MIP_LCOFIP;
                    mip_d = (mip_q & ~mask) | (csr_wdata & mask);
                end
                // performance counters
                escher_csr_pkg::CSR_MCYCLE:             csr_wdata_en     = '1;
                escher_csr_pkg::CSR_MINSTRET:           instret_d     = csr_wdata;

                escher_csr_pkg::CSR_MHPM_COUNTER_3,
                escher_csr_pkg::CSR_MHPM_COUNTER_4,
                escher_csr_pkg::CSR_MHPM_COUNTER_5,
                escher_csr_pkg::CSR_MHPM_COUNTER_6,
                escher_csr_pkg::CSR_MHPM_COUNTER_7,
                escher_csr_pkg::CSR_MHPM_COUNTER_8,
                escher_csr_pkg::CSR_MHPM_COUNTER_9,
                escher_csr_pkg::CSR_MHPM_COUNTER_10,
                escher_csr_pkg::CSR_MHPM_COUNTER_11,
                escher_csr_pkg::CSR_MHPM_COUNTER_12,
                escher_csr_pkg::CSR_MHPM_COUNTER_13,
                escher_csr_pkg::CSR_MHPM_COUNTER_14,
                escher_csr_pkg::CSR_MHPM_COUNTER_15,
                escher_csr_pkg::CSR_MHPM_COUNTER_16,
                escher_csr_pkg::CSR_MHPM_COUNTER_17,
                escher_csr_pkg::CSR_MHPM_COUNTER_18,
                escher_csr_pkg::CSR_MHPM_COUNTER_19,
                escher_csr_pkg::CSR_MHPM_COUNTER_20,
                escher_csr_pkg::CSR_MHPM_COUNTER_21,
                escher_csr_pkg::CSR_MHPM_COUNTER_22,
                escher_csr_pkg::CSR_MHPM_COUNTER_23,
                escher_csr_pkg::CSR_MHPM_COUNTER_24,
                escher_csr_pkg::CSR_MHPM_COUNTER_25,
                escher_csr_pkg::CSR_MHPM_COUNTER_26,
                escher_csr_pkg::CSR_MHPM_COUNTER_27,
                escher_csr_pkg::CSR_MHPM_COUNTER_28,
                escher_csr_pkg::CSR_MHPM_COUNTER_29,
                escher_csr_pkg::CSR_MHPM_COUNTER_30,
                escher_csr_pkg::CSR_MHPM_COUNTER_31: begin
                    hpm_write       = 1'b1;
                    hpm_wr_address  = (csr_addr.address - escher_csr_pkg::CSR_MHPM_COUNTER_3);
                    hpm_wdata       = csr_wdata;
                end

                escher_csr_pkg::CSR_MHPM_EVENT_3,
                escher_csr_pkg::CSR_MHPM_EVENT_4,
                escher_csr_pkg::CSR_MHPM_EVENT_5,
                escher_csr_pkg::CSR_MHPM_EVENT_6,
                escher_csr_pkg::CSR_MHPM_EVENT_7,
                escher_csr_pkg::CSR_MHPM_EVENT_8,
                escher_csr_pkg::CSR_MHPM_EVENT_9,
                escher_csr_pkg::CSR_MHPM_EVENT_10,
                escher_csr_pkg::CSR_MHPM_EVENT_11,
                escher_csr_pkg::CSR_MHPM_EVENT_12,
                escher_csr_pkg::CSR_MHPM_EVENT_13,
                escher_csr_pkg::CSR_MHPM_EVENT_14,
                escher_csr_pkg::CSR_MHPM_EVENT_15,
                escher_csr_pkg::CSR_MHPM_EVENT_16,
                escher_csr_pkg::CSR_MHPM_EVENT_17,
                escher_csr_pkg::CSR_MHPM_EVENT_18,
                escher_csr_pkg::CSR_MHPM_EVENT_19,
                escher_csr_pkg::CSR_MHPM_EVENT_20,
                escher_csr_pkg::CSR_MHPM_EVENT_21,
                escher_csr_pkg::CSR_MHPM_EVENT_22,
                escher_csr_pkg::CSR_MHPM_EVENT_23,
                escher_csr_pkg::CSR_MHPM_EVENT_24,
                escher_csr_pkg::CSR_MHPM_EVENT_25,
                escher_csr_pkg::CSR_MHPM_EVENT_26,
                escher_csr_pkg::CSR_MHPM_EVENT_27,
                escher_csr_pkg::CSR_MHPM_EVENT_28,
                escher_csr_pkg::CSR_MHPM_EVENT_29,
                escher_csr_pkg::CSR_MHPM_EVENT_30,
                escher_csr_pkg::CSR_MHPM_EVENT_31: begin
                    hpm_write       = 1'b1;
                    hpm_wr_event    = 1'b1;
                    hpm_wr_address  = (csr_addr.address - escher_csr_pkg::CSR_MHPM_EVENT_3);
                    hpm_wdata       = csr_wdata;
                end

                escher_csr_pkg::CSR_CYCLE,
                escher_csr_pkg::CSR_TIME,
                escher_csr_pkg::CSR_INSTRET,
                escher_csr_pkg::CSR_HPM_COUNTER_3,
                escher_csr_pkg::CSR_HPM_COUNTER_4,
                escher_csr_pkg::CSR_HPM_COUNTER_5,
                escher_csr_pkg::CSR_HPM_COUNTER_6,
                escher_csr_pkg::CSR_HPM_COUNTER_7,
                escher_csr_pkg::CSR_HPM_COUNTER_8,
                escher_csr_pkg::CSR_HPM_COUNTER_9,
                escher_csr_pkg::CSR_HPM_COUNTER_10,
                escher_csr_pkg::CSR_HPM_COUNTER_11,
                escher_csr_pkg::CSR_HPM_COUNTER_12,
                escher_csr_pkg::CSR_HPM_COUNTER_13,
                escher_csr_pkg::CSR_HPM_COUNTER_14,
                escher_csr_pkg::CSR_HPM_COUNTER_15,
                escher_csr_pkg::CSR_HPM_COUNTER_16,
                escher_csr_pkg::CSR_HPM_COUNTER_17,
                escher_csr_pkg::CSR_HPM_COUNTER_18,
                escher_csr_pkg::CSR_HPM_COUNTER_19,
                escher_csr_pkg::CSR_HPM_COUNTER_20,
                escher_csr_pkg::CSR_HPM_COUNTER_21,
                escher_csr_pkg::CSR_HPM_COUNTER_22,
                escher_csr_pkg::CSR_HPM_COUNTER_23,
                escher_csr_pkg::CSR_HPM_COUNTER_24,
                escher_csr_pkg::CSR_HPM_COUNTER_25,
                escher_csr_pkg::CSR_HPM_COUNTER_26,
                escher_csr_pkg::CSR_HPM_COUNTER_27,
                escher_csr_pkg::CSR_HPM_COUNTER_28,
                escher_csr_pkg::CSR_HPM_COUNTER_29,
                escher_csr_pkg::CSR_HPM_COUNTER_30,
                escher_csr_pkg::CSR_HPM_COUNTER_31,
                escher_csr_pkg::CSR_SCOUNTOVF:
                    update_access_exception = 1'b1;

                escher_csr_pkg::CSR_MEM_MAP_0,
                escher_csr_pkg::CSR_MEM_MAP_1,
                escher_csr_pkg::CSR_MEM_MAP_2,
                escher_csr_pkg::CSR_MEM_MAP_3,
                escher_csr_pkg::CSR_MEM_MAP_4,
                escher_csr_pkg::CSR_MEM_MAP_5,
                escher_csr_pkg::CSR_MEM_MAP_6,
                escher_csr_pkg::CSR_MEM_MAP_7,
                escher_csr_pkg::CSR_MEM_MAP_8,
                escher_csr_pkg::CSR_MEM_MAP_9,
                escher_csr_pkg::CSR_MEM_MAP_10,                
                escher_csr_pkg::CSR_MEM_MAP_11,                
                escher_csr_pkg::CSR_MEM_MAP_12,                
                escher_csr_pkg::CSR_MEM_MAP_13,                
                escher_csr_pkg::CSR_MEM_MAP_14,                
                escher_csr_pkg::CSR_MEM_MAP_15,
                escher_csr_pkg::CSR_IO_MAP_0,
                escher_csr_pkg::CSR_IO_MAP_1,
                escher_csr_pkg::CSR_IO_MAP_2,
                escher_csr_pkg::CSR_IO_MAP_3,
                escher_csr_pkg::CSR_IO_MAP_4,
                escher_csr_pkg::CSR_IO_MAP_5,
                escher_csr_pkg::CSR_IO_MAP_6,
                escher_csr_pkg::CSR_IO_MAP_7,
                escher_csr_pkg::CSR_IO_MAP_8,
                escher_csr_pkg::CSR_IO_MAP_9,
                escher_csr_pkg::CSR_IO_MAP_10,
                escher_csr_pkg::CSR_IO_MAP_11,
                escher_csr_pkg::CSR_IO_MAP_12,
                escher_csr_pkg::CSR_IO_MAP_13,
                escher_csr_pkg::CSR_IO_MAP_14,
                escher_csr_pkg::CSR_IO_MAP_15,
                escher_csr_pkg::CSR_IRQ_MAP_0,
                escher_csr_pkg::CSR_IRQ_MAP_1,
                escher_csr_pkg::CSR_IRQ_MAP_2,
                escher_csr_pkg::CSR_IRQ_MAP_3,
                escher_csr_pkg::CSR_IRQ_MAP_4,
                escher_csr_pkg::CSR_IRQ_MAP_5,
                escher_csr_pkg::CSR_IRQ_MAP_6,
                escher_csr_pkg::CSR_IRQ_MAP_7,
                escher_csr_pkg::CSR_IRQ_MAP_8,
                escher_csr_pkg::CSR_IRQ_MAP_9,
                escher_csr_pkg::CSR_IRQ_MAP_10,
                escher_csr_pkg::CSR_IRQ_MAP_11,
                escher_csr_pkg::CSR_IRQ_MAP_12,
                escher_csr_pkg::CSR_IRQ_MAP_13,
                escher_csr_pkg::CSR_IRQ_MAP_14,
                escher_csr_pkg::CSR_IRQ_MAP_15,
                escher_csr_pkg::FROM_HOST,
                escher_csr_pkg::CSR_HYPERRAM_CONFIG, 
                escher_csr_pkg::CSR_SPI_CONFIG, 
                escher_csr_pkg::CSR_CNM_CONFIG,
                escher_csr_pkg::TO_HOST: begin
                                        pcr_req_data_o = csr_wdata;

                end

                escher_csr_pkg::CSR_PMPCFG_0:;
                escher_csr_pkg::CSR_PMPCFG_1:;
                escher_csr_pkg::CSR_PMPCFG_2:;
                escher_csr_pkg::CSR_PMPCFG_3:;

                escher_csr_pkg::CSR_PMPADDR_0:;
                escher_csr_pkg::CSR_PMPADDR_1:;
                escher_csr_pkg::CSR_PMPADDR_2:;
                escher_csr_pkg::CSR_PMPADDR_3:;
                escher_csr_pkg::CSR_PMPADDR_4:;
                escher_csr_pkg::CSR_PMPADDR_5:;
                escher_csr_pkg::CSR_PMPADDR_6:;
                escher_csr_pkg::CSR_PMPADDR_7:;
                escher_csr_pkg::CSR_PMPADDR_8:;
                escher_csr_pkg::CSR_PMPADDR_9:;
                escher_csr_pkg::CSR_PMPADDR_10:;
                escher_csr_pkg::CSR_PMPADDR_11:;
                escher_csr_pkg::CSR_PMPADDR_12:;
                escher_csr_pkg::CSR_PMPADDR_13:;
                escher_csr_pkg::CSR_PMPADDR_14:;
                escher_csr_pkg::CSR_PMPADDR_15:;
                default: update_access_exception = 1'b1;
            endcase
        end

        // assign the temporal value to _d values to avoid multiples assign in the same cicle
        mstatus_d   = mstatus_int;
        mcause_d    = mcause_int;
        scause_d    = scause_int;
        mtval_d     = mtval_int;
        stval_d     = stval_int;
        mepc_d      = mepc_int;
        sepc_d      = sepc_int;

        // mark the floating point extension register as dirty 
        if (escher_csr_pkg::FP_PRESENT && (dirty_fp_state_csr)) begin
            mstatus_d.fs = escher_csr_pkg::Dirty_State;
        end
        // hardwire to zero if floating point extension is not present
        else if (!escher_csr_pkg::FP_PRESENT) begin
            mstatus_d.fs = escher_csr_pkg::Off_State;
        end

        // mark the vector extension register as dirty 
        if (escher_csr_pkg::V_PRESENT && (dirty_v_state_csr)) begin
            mstatus_d.vs = escher_csr_pkg::Dirty_State;
        end
        // hardwire to zero if vector extension is not present
        else if (!escher_csr_pkg::V_PRESENT) begin
            mstatus_d.vs = escher_csr_pkg::Off_State;
        end


        // -----------------
        // Interrupt Control
        // -----------------
        // we decode an interrupt the same as an exception, hence it will be taken if the instruction did not
        // throw any previous exception.
        // we have three interrupt sources: external interrupts, software interrupts, timer interrupts (order of precedence)
        // for two privilege levels: Supervisor and Machine Mode
        
        interrupt_cause_d = interrupt_cause_q;
        interrupt_d = 1'b0;
        interrupt_cause = 64'b0;
        ex_tval = 64'b0;
        // Machine Mode External Interrupt
        if (mip_q[escher_csr_pkg::M_EXT_INTERRUPT[5:0]] && mie_q[escher_csr_pkg::M_EXT_INTERRUPT[5:0]]) begin
            interrupt_cause = escher_csr_pkg::M_EXT_INTERRUPT;
        end
        // Machine Mode Software Interrupt
        else if (mip_q[escher_csr_pkg::M_SW_INTERRUPT[5:0]] && mie_q[escher_csr_pkg::M_SW_INTERRUPT[5:0]]) begin
            interrupt_cause = escher_csr_pkg::M_SW_INTERRUPT;
        end
        // Machine Timer Interrupt
        else if (mip_q[escher_csr_pkg::M_TIMER_INTERRUPT[5:0]] && mie_q[escher_csr_pkg::M_TIMER_INTERRUPT[5:0]]) begin
            interrupt_cause = escher_csr_pkg::M_TIMER_INTERRUPT;
        end
        // Supervisor External Interrupt
        else if (mie_q[escher_csr_pkg::S_EXT_INTERRUPT[5:0]] && (mip_q[escher_csr_pkg::S_EXT_INTERRUPT[5:0]] || irq_i)) begin
            interrupt_cause = escher_csr_pkg::S_EXT_INTERRUPT;
        end
        // Supervisor Software Interrupt
        else if (mie_q[escher_csr_pkg::S_SW_INTERRUPT[5:0]] && mip_q[escher_csr_pkg::S_SW_INTERRUPT[5:0]]) begin
            interrupt_cause = escher_csr_pkg::S_SW_INTERRUPT;
        end
        // Supervisor Timer Interrupt
        else if (mie_q[escher_csr_pkg::S_TIMER_INTERRUPT[5:0]] && mip_q[escher_csr_pkg::S_TIMER_INTERRUPT[5:0]]) begin
            interrupt_cause = escher_csr_pkg::S_TIMER_INTERRUPT;
        end
        // Local Count Overflow Interrupt
        else if (mip_q[escher_csr_pkg::LCOF_INTERRUPT[5:0]] && mie_q[escher_csr_pkg::LCOF_INTERRUPT[5:0]]) begin
            interrupt_cause = escher_csr_pkg::LCOF_INTERRUPT;
        end
        
        // if the priv is different of M or the mie is 1, the interrups are enable
        global_enable = ((mstatus_q.mie & (priv_lvl_o == escher_csr_pkg::PRIV_LVL_M))
                                    | (priv_lvl_o != escher_csr_pkg::PRIV_LVL_M));

        if (interrupt_cause[63] && global_enable) begin
            // However, if bit i in mideleg is set, interrupts are considered to be globally enabled if the hart’s current privilege
            // mode equals the delegated privilege mode (S or U) and that mode’s interrupt enable bit
            // (SIE or UIE in mstatus) is set, or if the current privilege mode is less than the delegated privilege mode.
            if (mideleg_q[interrupt_cause[5:0]]) begin
                if ((mstatus_q.sie && (priv_lvl_q == escher_csr_pkg::PRIV_LVL_S)) || (priv_lvl_q == escher_csr_pkg::PRIV_LVL_U)) begin
                    interrupt_d = 1'b1;
                    interrupt_cause_d = interrupt_cause;
                end
            end else begin
                interrupt_d = 1'b1;
                interrupt_cause_d = interrupt_cause;
            end
        end
        
        // a sfence is executed and a flush is needed
        if (flush_sfence) begin
            flush = 1'b1;
        end
        flush_o = flush;

        //Output connection
        interrupt_o = interrupt_q;
        interrupt_cause_o = interrupt_cause_q;
        // -----------------------
        // Manage Exception Stack
        // -----------------------
        // update exception CSRs
        // we got an exception update cause, pc and stval register
        trap_to_priv_lvl = escher_csr_pkg::PRIV_LVL_M;
        // Exception is taken and we are not in debug mode
        // exceptions in debug mode don't update any fields
        
        if (((ex_cause_i != escher_csr_pkg::DEBUG_REQUEST) && ex_i) || csr_xcpt) begin
            // do not flush, flush is reserved for CSR writes with side effects
            flush_o   = 1'b0;
            ex_tval = pc_i;
            //tval is the actual pc except for the data access exeptions
            if (ex_i && ex_cause_i inside {escher_csr_pkg::LD_ADDR_MISALIGNED, escher_csr_pkg::LD_ACCESS_FAULT, 
                                    escher_csr_pkg::ST_AMO_ADDR_MISALIGNED, escher_csr_pkg::ST_AMO_ACCESS_FAULT,
                                    escher_csr_pkg::LD_PAGE_FAULT, escher_csr_pkg::ST_AMO_PAGE_FAULT,
                                    escher_csr_pkg::INSTR_PAGE_FAULT, escher_csr_pkg::INSTR_ADDR_MISALIGNED}) begin
                ex_tval = w_data_core_i;
            end else if ((ex_i && (ex_cause_i == escher_csr_pkg::ILLEGAL_INSTR)) || (csr_xcpt && (csr_xcpt_cause == escher_csr_pkg::ILLEGAL_INSTR))) begin
                ex_tval = 64'b0;
	    end else if (csr_xcpt && (csr_xcpt_cause == escher_csr_pkg::BREAKPOINT)) begin
                ex_tval = pc_i;
            end else begin
                ex_tval = 64'b0;
            end
            // figure out where to trap to
            // a m-mode trap might be delegated if we are taking it in S mode
            // first figure out if this was an exception or an interrupt e.g.: look at bit 63
            // the cause register can only be 6 bits long (as we only support 64 exceptions)
            if ((ex_i &&((ex_cause_i[63] && mideleg_q[ex_cause_i[5:0]]) ||
                (~ex_cause_i[63] && medeleg_q[ex_cause_i[5:0]]))) ||
                (csr_xcpt && ((csr_xcpt_cause[63] && mideleg_q[csr_xcpt_cause[5:0]]) ||
                (~csr_xcpt_cause[63] && medeleg_q[csr_xcpt_cause[5:0]])))) begin
                // traps never transition from a more-privileged mode to a less privileged mode
                // so if we are already in M mode, stay there
                trap_to_priv_lvl = (priv_lvl_o == escher_csr_pkg::PRIV_LVL_M) ? escher_csr_pkg::PRIV_LVL_M : escher_csr_pkg::PRIV_LVL_S;
            end

            // trap to supervisor mode
            if (trap_to_priv_lvl == escher_csr_pkg::PRIV_LVL_S) begin
                // update sstatus
                mstatus_d.sie  = 1'b0;
                mstatus_d.spie = mstatus_q.sie;
                // this can either be user or supervisor mode
                mstatus_d.spp  = priv_lvl_q[0];
                // set cause
                scause_d       = csr_xcpt ? csr_xcpt_cause : ex_cause_i;
                // set epc
                sepc_d         = pc_i;
                // set mtval or stval
                // stval_d have a special case for a mecanisem of ariane. In DRAC stval_d = ex_i.tval.
                stval_d        = ex_tval;
            // trap to machine mode
            end else begin
                // update mstatus
                mstatus_d.mie  = 1'b0;
                mstatus_d.mpie = mstatus_q.mie;
                // save the previous privilege mode
                mstatus_d.mpp  = priv_lvl_q;
                mcause_d       = csr_xcpt ? csr_xcpt_cause : ex_cause_i;
                // set epc
                mepc_d         = pc_i;
                // set mtval or stval
                // stval_d have a special case for a mecanisem of ariane. In DRAC stval_d = ex_i.tval.
                mtval_d        = ex_tval;
            end
            
            priv_lvl_d = trap_to_priv_lvl;
        end
        // ------------------------------
        // Return from Environment
        // ------------------------------
        // When executing an xRET instruction, supposing xPP holds the value y, xIE is set to xPIE; the privilege
        // mode is changed to y; xPIE is set to 1; and xPP is set to U
        else if (mret) begin
            // return from exception, IF doesn't care from where we are returning
            eret_o = 1'b1;
            // return to the previous privilege level and restore all enable flags
            // get the previous machine interrupt enable flag
            mstatus_d.mie  = mstatus_q.mpie;
            // restore the previous privilege level
            priv_lvl_d     = mstatus_q.mpp;
            // set mpp to user mode
            mstatus_d.mpp  = escher_csr_pkg::PRIV_LVL_U;
            // set mpie to 1
            mstatus_d.mpie = 1'b1;
        end else if (sret && !((priv_lvl_q == escher_csr_pkg::PRIV_LVL_S) && mstatus_q.tsr)) begin
            // return from exception, IF doesn't care from where we are returning
            eret_o = 1'b1;
            // return the previous supervisor interrupt enable flag
            mstatus_d.sie  = mstatus_q.spie;
            // restore the previous privilege level
            priv_lvl_d     = escher_csr_pkg::priv_lvl_t'({1'b0, mstatus_q.spp});
            // set spp to user mode
            mstatus_d.spp  = 1'b0;
            // set spie to 1
            mstatus_d.spie = 1'b1;
        end

        // ------------------------------
        // MPRV - Modify Privilege Level
        // ------------------------------
        // Set the address translation at which the load and stores should occur
        // we can use the previous values since changing the address translation will always involve a pipeline flush
        if (mprv && (satp_q.mode == escher_csr_pkg::MODE_SV39) && (mstatus_q.mpp != escher_csr_pkg::PRIV_LVL_M)) begin
            en_ld_st_translation_d = 1'b1;
        end else begin // otherwise we go with the regular settings
            en_ld_st_translation_d = en_translation_o;
            //ld_st_priv_lvl_o = (mprv) ? mstatus_q.mpp : priv_lvl_o;
            //en_ld_st_translation_o = en_ld_st_translation_q;
        end
	
	ld_st_priv_lvl_o = (mprv) ? mstatus_q.mpp : priv_lvl_o;
	en_ld_st_translation_o = en_ld_st_translation_q;
        
    end

    assign csr_tval_o = ex_tval;

    // ---------------------------
    // CSR OP Select Logic
    // ---------------------------
    always_comb begin : csr_op_logic
        flush_sfence = 1'b0;
        csr_wdata = w_data_core_i;
        csr_we    = 1'b1;
        csr_read  = 1'b1;
        mret      = 1'b0;
        sret      = 1'b0;

        unique case (rw_cmd_i)
            4'b0001: csr_wdata = w_data_core_i;                // Write and Read
            4'b0010: csr_wdata = w_data_core_i | csr_rdata;    // Set and Read
            4'b0011: csr_wdata = (~w_data_core_i) & csr_rdata; // Clear and Read
            4'b0101: csr_we    = 1'b0;                         // Read only
            4'b1000: csr_read  = 1'b0;                         // Write only
            default: begin
                csr_we   = 1'b0;
                csr_read = 1'b0;
            end
        endcase
        if (insn_sret) begin
            // the return should not have any write or read side-effects
            sret     = 1'b1; // signal a return from supervisor mode
        end else if (insn_mret) begin
            // the return should not have any write or read side-effects
            mret     = 1'b1; // signal a return from machine mode
        end else if (insn_sfence_vm) begin // flush_o can not be changed here, it changes in csr_update process
            flush_sfence = 1'b1;
        end
        // if we are violating our privilges do not update the architectural state
        if (privilege_violation) begin
            csr_we = 1'b0;
            csr_read = 1'b0;
        end
    end

    // -----------------
    // Privilege Check
    // -----------------
    always_comb begin : privilege_check
        privilege_violation = 1'b0;
        // if we are reading or writing, check for the correct privilege level this has
        // precedence over interrupts
        if (rw_cmd_i inside {4'b0001, 4'b0010, 4'b0011, 4'b0101, 4'b1000}) begin // inside of: rw, set, clear, read, write
            if ((escher_csr_pkg::priv_lvl_t'(priv_lvl_o & csr_addr.csr_decode.priv_lvl) != csr_addr.csr_decode.priv_lvl)) begin
                privilege_violation = 1'b1;
            end
            // check access to debug mode only CSRs 
            if (rw_addr_i[11:4] == 8'h7b ) begin
                privilege_violation = 1'b1;
            end
            if (csr_addr.address inside {[escher_csr_pkg::CSR_CYCLE:escher_csr_pkg::CSR_HPM_COUNTER_31]}) begin
                unique case (priv_lvl_o)
                    escher_csr_pkg::PRIV_LVL_M: privilege_violation = 1'b0;
                    escher_csr_pkg::PRIV_LVL_S: privilege_violation = ~mcounteren_q[csr_addr.address[4:0]];
                    escher_csr_pkg::PRIV_LVL_U: privilege_violation = ~mcounteren_q[csr_addr.address[4:0]] & ~scounteren_q[csr_addr.address[4:0]];
                endcase
            end
        end
        //checks the level of the system instruction or sfence with tvm = 1 or sret and tsr = 1
        if ((system_insn && !priv_sufficient ) || (insn_sfence_vm && (priv_lvl_q == escher_csr_pkg::PRIV_LVL_S) && mstatus_q.tvm)
            || (insn_sret && (priv_lvl_q == escher_csr_pkg::PRIV_LVL_S) && mstatus_q.tsr)) begin
             privilege_violation = 1'b1;
        end
    end

    // -------------------
    // Vector instruccions excecution
    // -------------------
    always_comb begin : vsetvl_ctrl

        vtype_new = rw_addr_i[10:0];
        // new vlmax depending on the vtype config
        vlmax = ((escher_csr_pkg::VLEN << vtype_new[1:0]) >> 3) >> vtype_new[4:2];
        dirty_v_state_csr = 1'b0;
        update_access_exception_vs = 1'b0;

        if (vsetvl_insn) begin
            if (mstatus_q.vs == escher_csr_pkg::Off_State) begin
                update_access_exception_vs = 1'b1;
                // default, keeps the old value
                vl_d = vl_q;
                vtype_d = vtype_q;
            end else begin
                dirty_v_state_csr = 1'b1;
                // vl assignation depending on the AVL respect VLMAX
                if (rw_cmd_i == 3'b111) begin //vsetvl with x0
                    if (w_data_core_i == 64'b1) begin
                        vl_d = vl_q;
                    end else begin
                        vl_d = vlmax;
                    end
                end else if (vlmax >= w_data_core_i) begin
                    vl_d = w_data_core_i;
                end else if ((vlmax<<1) >= w_data_core_i) begin
                    vl_d = w_data_core_i>>(1 + w_data_core_i[0]);   //TODO: check this, before was  w_data_core_i>>1 + w_data_core_i[0];
                end else begin
                    vl_d = vlmax;
                end
                // vtype assignation
                if (vtype_new[10:4] != 6'b0) begin
                    vtype_d = {1'b1,63'b0};
                end else begin
                    vtype_d = {'0,vtype_new};
                end  
            end              
        end else  begin
            // default, keeps the old value
            vl_d = vl_q;
            vtype_d = vtype_q;
        end
    end

    assign vpu_csr_o = {vtype_q[63], vtype_q[4:0], fcsr_q[7:5], 2'b0, vl_q[14:0], 14'b0};

    // ----------------------
    // CSR Exception Control
    // ----------------------
    assign csr_xcpt_o = csr_xcpt;
    assign csr_xcpt_cause_o = csr_xcpt_cause;

    always_comb begin : exception_ctrl
        csr_xcpt_cause = 64'b0;
        csr_xcpt = 1'b0;
        // ----------------------------------
        // Illegal Access (decode exception)
        // ----------------------------------
        // we got an exception in one of the processes above
        // throw an illegal instruction exception
        if (update_access_exception || update_access_exception_vs || read_access_exception) begin
            csr_xcpt_cause = escher_csr_pkg::ILLEGAL_INSTR;
            // we don't set the tval field as this will be set by the commit stage
            // this spares the extra wiring from commit to CSR and back to commit
            csr_xcpt = 1'b1;
        end else if (privilege_violation) begin
          csr_xcpt_cause = escher_csr_pkg::ILLEGAL_INSTR;
          csr_xcpt = 1'b1;
        end else if (insn_call) begin
            csr_xcpt_cause = escher_csr_pkg::USER_ECALL + priv_lvl_q;
            csr_xcpt = 1'b1;
        end else if (insn_break) begin
            csr_xcpt_cause = escher_csr_pkg::BREAKPOINT;
            csr_xcpt = 1'b1;
        end else if (insn_wfi && mstatus_q.tw) begin
            csr_xcpt_cause = escher_csr_pkg::ILLEGAL_INSTR;
            csr_xcpt = 1'b1;
        end
    end

    // -------------------
    // Wait for Interrupt
    // -------------------
    always_comb begin : wfi_ctrl
        // wait for interrupt register
        wfi_d = wfi_q;
        // if there is any interrupt pending un-stall the core
        if ((|mip_q) || irq_i) begin
            wfi_d = 1'b0;
        // or alternatively if there is no exception pending and we are not in debug mode wait here
        // for the interrupt
        end else if (insn_wfi && !ex_i) begin
            wfi_d = 1'b1;
        end
    end

    // -------------------
    // PCR contol logic
    // -------------------

    always_comb begin : pcr_ctrl
        //whait until a response from the pcs
        cpu_ren = 1'b0;
        pcr_wait_resp_d = pcr_wait_resp_q;

        // determine if the cpu requests a read of a csr
        if (rw_cmd_i inside {4'b0001, 4'b0010, 4'b0011, 4'b0101}) begin //rw, set, clear, read
            cpu_ren = 1'b1;
        end

        // requests to pcr are valid when the pcr is ready, there aren't any exceptions and the 
        pcr_req_valid = cpu_ren & pcr_addr_valid & priv_sufficient & ~csr_xcpt & ~pcr_wait_resp_d;
        

        if (pcr_req_valid && (!pcr_resp_valid_i || (pcr_resp_core_id_i != core_id_i) )) pcr_wait_resp_d = 1'b1;
        else if (pcr_resp_valid_i && (pcr_resp_core_id_i == core_id_i)) pcr_wait_resp_d = 1'b0;
        
        //pcr requests outputs connections
        pcr_req_addr_o = csr_addr;
        pcr_req_we_o = rw_cmd_i;
        pcr_req_core_id_o = core_id_i;
        pcr_req_valid_o = pcr_req_valid;
    end

    // -------------------
    // CPU actions induced by the csr
    // -------------------
    assign csr_replay_o = pcr_req_valid & !pcr_req_ready_i; // pcr write but not ready
    assign csr_stall_o = wfi_q | // or waiting pcr response 
                  (pcr_wait_resp_d & (~pcr_resp_valid_i | (pcr_resp_core_id_i != core_id_i)));


    // output assignments dependent on privilege mode
    always_comb begin : priv_output
        // vectorized trap addres
        trap_vector_base[63:8] = mtvec_q[63:8];
        trap_vector_base[7:2] = ((mtvec_q[1:0] == 2'b0) || csr_xcpt || !ex_cause_i[63]) ? mtvec_q[7:2] : csr_xcpt ? (mtvec_q[7:2] + csr_xcpt_cause[5:0]) : (mtvec_q[7:2] + ex_cause_i[5:0]);
        trap_vector_base[1:0] = 2'b0;
        // output user mode stvec
        if (trap_to_priv_lvl == escher_csr_pkg::PRIV_LVL_S) begin
            // vectorized trap addres
            trap_vector_base[63:8] = stvec_q[63:8];
            trap_vector_base[7:2] = ((stvec_q[1:0] == 2'b0) || csr_xcpt || !ex_cause_i[63]) ? stvec_q[7:2] : csr_xcpt ? (mtvec_q[7:2] + csr_xcpt_cause[5:0]) : (mtvec_q[7:2] + ex_cause_i[5:0]);
            trap_vector_base[1:0] = 2'b0;
        end

        evec_o = mepc_q; // we are returning from machine mode, so take the mepc register
        
        if (ex_i || csr_xcpt_o) begin // an exception is detected in the core and it is send the trap address
            evec_o = trap_vector_base;
        end else if (sret) begin // we are returning from supervisor mode, so take the sepc register
            evec_o = sepc_q;
        end
    end

    // -------------------
    // HPM 
    // -------------------
    escher_hpm_counters #(
        .COUNTER_EVENTS (COUNTER_EVENTS)
    ) hpm_counters_inst (
        .clk_i          (clk_i),                                          //! System clock signal.
        .rstn_i         (rstn_i),                                         //! System reset signal (active low).  

        .event_i        (hpm_events_i),   

        .wr_en_i        (hpm_write),
        .wr_hpm_event_i (hpm_wr_event),
        .wr_hpm_id_i    (hpm_wr_address),
        .wr_hpm_data_i  (hpm_wdata),   

        .rd_en_i        (hpm_read),
        .rd_hpm_event_i (hpm_rd_event),
        .rd_hpm_id_i    (hpm_rd_addr),
        .rd_hpm_data_o  (hpm_rdata)
    );

    // -------------------
    // Output Assignments
    // -------------------
    always_comb begin
        // When the SEIP bit is read with a CSRRW, CSRRS, or CSRRC instruction, the value
        // returned in the rd destination register contains the logical-OR of the software-writable
        // bit and the interrupt signal from the interrupt controller.
	if (vsetvl_insn) begin
        	r_data_core_o = vl_d;
	end else begin
		r_data_core_o = csr_rdata;
        	unique case (csr_addr.address)
        	    escher_csr_pkg::CSR_MIP: r_data_core_o = csr_rdata | ({63'b0,irq_i} << escher_csr_pkg::IRQ_S_EXT);
        	    // in supervisor mode we also need to check whether we delegated this bit
        	    escher_csr_pkg::CSR_SIP: begin
        	        r_data_core_o = csr_rdata
        	                    | ({63'b0,(irq_i & mideleg_q[escher_csr_pkg::IRQ_S_EXT])} << escher_csr_pkg::IRQ_S_EXT);
        	    end
                default:;
                endcase
	end
    end
    // fcrs assigments
    assign status_o = mstatus_q;
    // in debug mode we execute with privilege level M
    assign priv_lvl_o       = priv_lvl_q;
    // FPU outputs

    assign fcsr_rm_o        = fcsr_q.frm;
    assign fcsr_fs_o        = mstatus_q.fs;

    assign bke_queues_inorder_o = uarch_cfg_csr_q.bke_queues_inorder;
    `ifdef FPGA
        assign ila_dbg_cfg_o        = ila_dbg_cfg_q;
        assign ila_cnd_pc_o         = ila_cnd_pc_q;
        assign ila_cnd_maddr_o      = ila_cnd_maddr_q;
        assign ila_cnd_time_o       = ila_cnd_time_q;
        assign ila_cnd_mask_o       = ila_cnd_mask_q;
    `endif

    // VPU outputs
    assign vcsr_vs_o        = mstatus_q.vs;


    // MMU outputs 
    assign satp_o          = satp_q;

    // we support bare memory addressing and SV39
    assign en_translation_o = ((satp_q.mode == 4'h8) && (priv_lvl_o != escher_csr_pkg::PRIV_LVL_M))
                              ? 1'b1
                              : 1'b0;

    // mprv assignation
    assign mprv             = mstatus_q.mprv;

    // timer output assign
    assign reg_time_d = time_i;

    // sequential process
    always_ff @(posedge clk_i, negedge rstn_i) begin
        if (!rstn_i) begin
            priv_lvl_q             <= escher_csr_pkg::PRIV_LVL_M;
            // floating-point registers
            fcsr_q                 <= 64'b0;
            // uArch config
            uarch_cfg_csr_q        <= 64'b0;
            // ILA config
            `ifdef FPGA
            ila_dbg_cfg_q          <= ila_dbg_cfg_init;
            ila_cnd_pc_q           <= 64'b0;
            ila_cnd_maddr_q        <= 64'b0;
            ila_cnd_time_q         <= 64'b0;
            ila_cnd_mask_q         <= 64'b0;
            `endif
            // machine mode registers
            mstatus_q              <= 64'b011000000000;
            // set to boot address + direct mode + 4 byte offset which is the initial trap
            mtvec_rst_load_q       <= 1'b1;
            mtvec_q                <= 64'b0;
            medeleg_q              <= 64'b0;
            mideleg_q              <= 64'b0;
            mip_q                  <= 64'b0;
            mie_q                  <= 64'b0;
            mepc_q                 <= 64'b0;
            mcause_q               <= 64'b0;
            mcounteren_q           <= 32'b0;
            mcountinhibit_q        <= 32'b0;  // Not assigned again
            mscratch_q             <= 64'b0;
            mtval_q                <= 64'b0;
            // supervisor mode registers
            sepc_q                 <= 64'b0;
            scause_q               <= 64'b0;
            stvec_q                <= 64'b0;
            scounteren_q           <= 32'b0;
            sscratch_q             <= 64'b0;
            stval_q                <= 64'b0;
            satp_q                 <= 64'b0;
            // timer and counters
            cycle_q                <= 64'b0;
            instret_q              <= 64'b0;
            scountovf_q            <= 32'b0;
            // aux registers
            en_ld_st_translation_q <= 1'b0;
            // wait for interrupt
            wfi_q                  <= 1'b0;
            //PCR assigments
            pcr_wait_resp_q <= 1'b0;
            reg_time_q <= 64'b0;

            //Interrupt assigments
            interrupt_q <= 1'b0;
            interrupt_cause_q <= 64'b0;

            // Vector extension
            vl_q              <= 64'b0;
            vtype_q           <= {1'b1, 63'b0};
        end else begin
            priv_lvl_q             <= priv_lvl_d;
            // floating-point registers
            fcsr_q                 <= fcsr_d;
            // uArch config
            uarch_cfg_csr_q        <= uarch_cfg_csr_d;

            // ILA config
            `ifdef FPGA
            ila_dbg_cfg_q          <= ila_dbg_cfg_d;
            ila_cnd_pc_q           <= ila_cnd_pc_d;
            ila_cnd_maddr_q        <= ila_cnd_maddr_d;
            ila_cnd_time_q         <= ila_cnd_time_d;
            ila_cnd_mask_q         <= ila_cnd_mask_d;
            `endif

            // machine mode registers
            mstatus_q              <= mstatus_d;
            mtvec_rst_load_q       <= 1'b0;
            mtvec_q                <= mtvec_d;
            medeleg_q              <= medeleg_d;
            mideleg_q              <= mideleg_d;
            mip_q                  <= mip_d;
            mie_q                  <= mie_d;
            mepc_q                 <= mepc_d;
            mcause_q               <= mcause_d;
            mcounteren_q           <= mcounteren_d;

            mcountinhibit_q        <= mcountinhibit_d;  // Was not assigned again and is not in the spec, added to avoid latch, TODO: VERIFY

            mscratch_q             <= mscratch_d;
            mtval_q                <= mtval_d;
            // supervisor mode registers
            sepc_q                 <= sepc_d;
            scause_q               <= scause_d;
            stvec_q                <= stvec_d;
            scounteren_q           <= scounteren_d;
            sscratch_q             <= sscratch_d;
            stval_q                <= stval_d;
            satp_q                 <= satp_d;
            // timer and counters
            cycle_q                <= cycle_d;
            instret_q              <= instret_d;
            scountovf_q            <= scountovf_d;
            // aux registers
            en_ld_st_translation_q <= en_ld_st_translation_d;
            // wait for interrupt
            wfi_q                  <= wfi_d;
            // PCR assigments
            pcr_wait_resp_q <= pcr_wait_resp_d;
            reg_time_q <= reg_time_d; 

            //Interrupt assigments
            interrupt_q <= interrupt_d;
            interrupt_cause_q <= interrupt_cause_d;

            // Vector extension
            vl_q                    <= vl_d;
            vtype_q                 <= vtype_d;
        end
    end

    `ifdef SIM_COMMIT_LOG
    `ifdef SIM_COMMIT_LOG_DPI
        import "DPI-C" function void csr_change (input longint unsigned addr, input longint unsigned value);
        always_ff @(negedge clk_i) begin
            automatic logic [63:0] mstatus_fix, mstatus_clear ,mstatus_set;

            mstatus_clear = escher_csr_pkg::MSTATUS_UXL | escher_csr_pkg::MSTATUS_SXL | escher_csr_pkg::MSTATUS64_SD;
            mstatus_set =   ((mstatus_d.xs == escher_csr_pkg::Dirty_State) | (mstatus_d.fs == escher_csr_pkg::Dirty_State) | (mstatus_d.vs == escher_csr_pkg::Dirty_State))<<63 |
                            escher_csr_pkg::XLEN_64 << 32|
                            escher_csr_pkg::XLEN_64 << 34;
            mstatus_fix = (mstatus_d & ~mstatus_clear) | mstatus_set;
            if (rstn_i & (|torture_dpi_we_i)) begin
                // CSRs which can change due to side-effects etc

                // For some reason mstatus is updated in multiple cycles, but at commit we need the value that will be set later
                // This might break in some random tests.....
                if (mstatus_q != mstatus_fix) csr_change(escher_csr_pkg::CSR_MSTATUS, mstatus_fix);

                if (fcsr_flags_valid_i && fcsr_flags_bits_i) csr_change(escher_csr_pkg::CSR_FFLAGS, fcsr_d.fflags);

                // CSRs which only change when written to
                if (csr_we) begin
                    case(csr_addr)
                        escher_csr_pkg::CSR_MSTATUS, escher_csr_pkg::CSR_SSTATUS:
                            ; // Covered by previous mstatus check
                        escher_csr_pkg::CSR_MTVEC: csr_change(csr_addr, mtvec_d);
                        escher_csr_pkg::CSR_MEPC: csr_change(csr_addr, mepc_d);
                        escher_csr_pkg::CSR_MCAUSE: csr_change(csr_addr, mcause_d);
                        escher_csr_pkg::CSR_MSCRATCH: csr_change(csr_addr, mscratch_d);
                        escher_csr_pkg::CSR_MEDELEG: csr_change(csr_addr, medeleg_d);
                        escher_csr_pkg::CSR_SIE,
                        escher_csr_pkg::CSR_MIE: csr_change(escher_csr_pkg::CSR_MIE, mie_d);
                        escher_csr_pkg::CSR_SATP: csr_change(csr_addr, satp_d);
                        escher_csr_pkg::CSR_STVEC: csr_change(csr_addr, stvec_d);
                        escher_csr_pkg::CSR_SSCRATCH: csr_change(csr_addr, sscratch_d);
                        escher_csr_pkg::CSR_SEPC: csr_change(csr_addr, sepc_d);
                        escher_csr_pkg::CSR_MISA: csr_change(csr_addr, escher_csr_pkg::ISA_CODE);
                        default: csr_change(csr_addr, csr_wdata);
                    endcase
                end
            end
        end
        `endif
    `endif

    //-------------
    // Assertions
    //-------------
    //pragma translate_off
    `ifndef VERILATOR
        // check that eret and ex are never valid together
        assert property (
          @(posedge clk_i) !(eret_o && ex_i))
        else begin $error("eret and exception should never be valid at the same time"); end
    `endif
    //pragma translate_on
endmodule
